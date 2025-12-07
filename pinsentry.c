// PINSentry

#include <stdio.h>
#include <string.h>
#include <popt.h>
#include <time.h>
#include <sys/time.h>
#include <stdlib.h>
#include <ctype.h>
#include <err.h>
#include <assert.h>
#include <unistd.h>
#include <libusb-1.0/libusb.h>

int debug = 0;                  // Debug
int dump = 0;                   // Dump USB level messages

void
dumphex (const char *what, int len, const uint8_t *data)
{                               // Debug message dump
   fprintf (stderr, "%s:\t", what);
   while (len--)
      fprintf (stderr, "%02X", *data++);
   fprintf (stderr, "\n");
}

// Low level USB

libusb_device_handle *
usb_connect (const char *usbid)
{
   libusb_device_handle *usb = NULL;
   enum libusb_error r;
   if ((r = libusb_init (NULL)))
      errx (1, "Cannot init USB: %s", libusb_strerror (r));
   int vendor,
     product;
   if (!usbid || !*usbid || sscanf (usbid, "%X:%X", &vendor, &product) != 2)
      errx (1, "USB setting is vendor:product");
   usb = libusb_open_device_with_vid_pid (NULL, vendor, product);
   if (!usb)
      return usb;               // Cannot open
   //
   if ((r = libusb_set_configuration (usb, 0)))
      errx (1, "Cannot set USB config: %s", libusb_strerror (r));

   struct libusb_device *dev = libusb_get_device (usb);
   if (!dev)
      errx (1, "No device handle?");

   if ((r = libusb_set_auto_detach_kernel_driver (usb, 1)))
      errx (1, "Cannot detach USB: %s", libusb_strerror (r));

   if ((r = libusb_claim_interface (usb, 0)))   // Interface 0
      errx (1, "Cannot claim USB: %s", libusb_strerror (r));

   // TODO finding endpoint identifiers somehow

   return usb;
}

void
usb_disconnect (libusb_device_handle *usb)
{
   assert (usb);
   libusb_release_interface (usb, 0);   // Interface 0
   libusb_close (usb);
   libusb_exit (NULL);
}

uint8_t
usb_int_rx (libusb_device_handle *usb, uint8_t max, uint8_t *rx, int to)
{                               // Wait for interrupt
   assert (usb);
   assert (rx);
   assert (max);
   int l = 0;
   int r = libusb_interrupt_transfer (usb, 0x82, rx, max, &l, to);
   if (r == LIBUSB_ERROR_TIMEOUT)
      return 0;
   if (r)
      errx (1, "Interrupt failed: %s", libusb_strerror (r));
   return l;
}

uint32_t
usb_bulk_txn (libusb_device_handle *usb, int txlen, const uint8_t *tx, int rxmax, uint8_t *rx)
{                               // Send a tx and receive an rx over bulk endpoints - return rx len
   assert (usb);
   assert (tx);
   assert (rx);
   static uint8_t rxpending[1024];      // Waiting data as we may not get in clean packets
   static int rxq = 0;
   int to = 0;                  // ms or 0 for unlimited
   enum libusb_error r;
   {                            // Tx
      if (dump)
         dumphex ("Tx", txlen, tx);
      int txp = 0;
      while (txp < txlen)
      {
         int try = 10,
            txsize = 0;
         while (try--)
         {                      // Send to endpoint 1
            txsize = 64;
            if (txlen - txp < txsize)
               txsize = txlen - txp;
            if ((r = libusb_bulk_transfer (usb, 1, ((uint8_t *) tx) + txp, txsize, &txsize, to)) != LIBUSB_ERROR_PIPE)
               break;
            libusb_clear_halt (usb, 1);
         }
         if (r)
            errx (1, "USB failed: %s", libusb_strerror (r));
         txp += txsize;
      }
   }

   int rxp = 0,
      rxe = 10;
   while (rxp < rxe)
   {
      int try = 10,
         rxsize = 0;
      while (try--)
      {
         rxsize = 64;
         if (rxe - rxp < rxsize)
            rxsize = rxe - rxp;
         if (!rxq)
         {
            if ((r = libusb_bulk_transfer (usb, 0x81, rxpending, sizeof (rxpending), &rxq, to)) == LIBUSB_ERROR_PIPE)
            {
               libusb_clear_halt (usb, 0x81);
               continue;
            }
         }
         if (!rxq)
            errx (1, "USB Rx queue failed");
         // Copy from queue
         if (rxsize > rxq)
            rxsize = rxq;
         memcpy (rx + rxp, rxpending, rxsize);
         if (rxsize < rxq)
            memmove (rxpending, rxpending + rxsize, rxq - rxsize);
         rxq -= rxsize;
         break;
      }
      if (r)
         errx (1, "USB failed: %s", libusb_strerror (r));
      rxp += rxsize;
      if (rxp == 10 && (rx[7] & 0x80))
      {                         // Waiting
         if (dump)
            dumphex ("Rx(wait)", rxp, rx);
         rxp = 0;
      }
      if (rxp == 10 && (rxe = 10 + rx[1] + (rx[2] << 8) + (rx[3] << 16) + (rx[4] << 24)) > rxmax)
      {
         warnx ("Rx too long in usb_bulk_txn %u>%u", rxe, rxmax);
         return 0;
      }
   }
   if (dump)
      dumphex ("Rx", rxp, rx);
   return rxp;
}

// Low level CCID
uint32_t
ccid_txn (libusb_device_handle *usb, uint8_t txtype, int txlen, uint8_t *tx, uint8_t rxtype, int rxmax, uint8_t *rx)
{                               // Send a CCID transaction, return length. tx and rx are expected to include 10 byte CCID header but tx[0-6] are filled in here
   assert (usb);
   assert (tx);
   assert (rx);
   assert (txlen >= 10);
   assert (rxmax >= 10);
   static uint8_t seq = 0;      // Message sequence
   tx[0] = txtype;
   int l = txlen - 10;
   tx[1] = l;                   // Len of payload
   tx[2] = l >> 8;
   tx[3] = l >> 16;
   tx[4] = l >> 24;
   tx[5] = 0;                   // Slot
   tx[6] = seq++;
   l = usb_bulk_txn (usb, txlen, tx, rxmax, rx);
   if (l < 10)
      warnx ("Bad CCID txn rx len %u", l);
   else if (*rx != rxtype)
      warnx ("Bad CCID txn rx type %02X/%02X", rxtype, *rx);
   else if (l != (rx[1] | (rx[2] << 8) | (rx[3] << 16) | (rx[4] << 24)) + 10)
      warnx ("Bad CCID txn rx len %u/%u", l, (rx[1] | (rx[2] << 8) | (rx[3] << 16) | (rx[4] << 24)) + 10);
   else if (rx[5] != tx[5])
      warnx ("Bad CCID txn slot rx %u/%u", tx[5], rx[5]);
   else if (rx[6] != tx[6])
      warnx ("Bad CCID txn seq rx %u/%u", tx[6], rx[6]);
   return l;
}

// Higher level CCID

typedef enum
{
   card_active,
   card_inactive,
   card_missing,
   card_error
} card_status_t;

card_status_t
card_status (libusb_device_handle *usb)
{                               // Get card status
   assert (usb);
   uint8_t tx[10] = { 0 };
   uint8_t rx[10];
   if (ccid_txn (usb, 0x65, sizeof (tx), tx, 0x81, sizeof (rx), rx) < 10)
      return card_error;
   return (rx[7] & 3);
}

uint8_t
ccid_power_on (libusb_device_handle *usb, float voltage, int max, uint8_t *atr)
{                               // Power on, get ATR, and return len (0 failure)
   assert (usb);
   uint8_t tx[10] = { 0 };      // 0 is auto voltage
   if (voltage == 5)
      tx[7] = 1;
   else if (voltage == 3.0 || voltage == 3.3)
      tx[7] = 2;                // Is 3.3 valid ?
   else if (voltage == 1.8)
      tx[7] = 3;
   uint8_t rx[266];
   uint32_t l;
   if ((l = ccid_txn (usb, 0x62, sizeof (tx), tx, 0x80, sizeof (rx), rx)) < 10)
      return 0;
   l -= 10;
   if (atr)
   {
      if (l > max)
      {
         warnx ("ATR not enough space %u/%u", max, l);
         return 0;
      }
      memcpy (atr, rx + 10, l);
      if (*atr == 0x3B && (atr[1] & 0x10))
      {                         // Send rate change
         uint8_t tx[15] = { 0 };
         tx[10] = atr[2];
         if (ccid_txn (usb, 0x61, sizeof (tx), tx, 0x82, sizeof (rx), rx) < 10)
         {
            warnx ("Rate chaneg failed");
            return 0;
         }
      }
   }
   return l;
}

card_status_t
ccid_power_off (libusb_device_handle *usb)
{
   assert (usb);
   uint8_t tx[10] = { 0 };
   uint8_t rx[10] = { 0 };
   if (ccid_txn (usb, 0x63, sizeof (tx), tx, 0x81, sizeof (rx), rx) < 10)
      return card_error;
   return (rx[7] & 3);
}

uint16_t
card_txn (libusb_device_handle *usb, uint16_t txlen, const uint8_t *tx, uint16_t rxmax, uint8_t *rx)
{                               // Transfer to card and get response (status at end), return len
   if (debug)
      dumphex ("CardTx", txlen, tx);
   uint8_t txbuf[10 + 5 + 256] = { 0 }; // Allow 256 byte message after 5 byte command
   uint8_t rxbuf[10 + 256 + 2] = { 0 }; // Allow 256 byte response with 2 byte status
   txbuf[7] = 1;                // block wait time
   memcpy (txbuf + 10, tx, txlen);
   int l;
   if ((l = ccid_txn (usb, 0x6F, 10 + txlen, txbuf, 0x80, sizeof (rxbuf), rxbuf)) < 10)
      return 0;
   l -= 10;
   if (l > rxmax)
   {
      warnx ("Response too long %u/%u", l, rxmax);
      return 0;
   }
   memcpy (rx, rxbuf + 10, l);
   if (debug)
      dumphex ("CardRx", l, rx);
   return l;
}

// Card functions

uint16_t
select_file (libusb_device_handle *usb, uint8_t cla, uint8_t p1, uint8_t p2, uint8_t len, uint8_t *fn)
{                               // Select file, return status
   assert (usb);
   assert (len);
   assert (fn);
   uint8_t tx[5 + 7] = { cla, 0xA4, p1, p2, len };
   assert (len <= sizeof (tx) - 5);
   memcpy (tx + 5, fn, len);
   uint8_t rx[2];
   uint16_t res = card_txn (usb, 5 + len, tx, sizeof (rx), rx);
   if (res != 2)
      return 0;
   return (rx[0] << 8) | rx[1];
}

uint16_t
get_response (libusb_device_handle *usb, uint8_t len, uint16_t max, uint8_t *rx)
{                               // Get response
   assert (usb);
   assert (rx);
   assert (max >= len + 2);
   uint8_t tx[] = { 0x00, 0xC0, 0x00, 0x00, len };
   uint16_t l = card_txn (usb, sizeof (tx), tx, max, rx);
   if (l != len + 2)
   {
      warnx ("Bad get response");
      return 0;
   }
   if (rx[l - 2] >> 4 != 9)
      warnx ("Bad get response status %02X%02X", rx[l - 2], rx[l - 1]);
   return l;
}

uint16_t
read_file (libusb_device_handle *usb, uint8_t p1, uint8_t p2, uint8_t len, uint16_t max, uint8_t *rx)
{
   assert (usb);
   assert (rx);
   assert (max >= len + 2);
   uint8_t tx[] = { 0x00, 0xB2, p1, p2, len };
   uint16_t l = card_txn (usb, sizeof (tx), tx, max, rx);

}

int
main (int argc, const char *argv[])
{
   float voltage = 3;
   const char *port = "303a:0000";
   const char *pin = NULL;
   int pan = 0;
   int identify = 0;
   const char *respond = NULL;
   const char *sign = NULL;
   poptContext optCon;          // context for parsing command-line options
   {                            // POPT
      const struct poptOption optionsTable[] = {
         {"pan", 'P', POPT_ARG_NONE, &pan, 0, "Get PAN"},
         {"identify", 'I', POPT_ARG_NONE, &identify, 0, "Identify"},
         {"respond", 'R', POPT_ARG_STRING, &respond, 0, "Respond", "XXXXXXXX"},
         {"sign", 'S', POPT_ARG_STRING, &sign, 0, "Sign", "XXXXXXXX+XXX.XX"},
         {"pin", 'p', POPT_ARG_STRING, &pin, 0, "PIN", "XXXX"},
         {"usb", 'u', POPT_ARG_STRING | POPT_ARGFLAG_SHOW_DEFAULT, &port, 0, "USB Device", "VVVV:PPPP"},
         {"voltage", 'V', POPT_ARG_FLOAT | POPT_ARGFLAG_SHOW_DEFAULT, &voltage, 0, "Voltage", "V"},
         {"debug", 'v', POPT_ARG_NONE, &debug, 0, "Debug"},
         {"dump", 0, POPT_ARG_NONE, &dump, 0, "Debug USB data"},
         POPT_AUTOHELP {}
      };

      optCon = poptGetContext (NULL, argc, argv, optionsTable, 0);
      //poptSetOtherOptionHelp (optCon, "");

      int c;
      if ((c = poptGetNextOpt (optCon)) < -1)
         errx (1, "%s: %s\n", poptBadOption (optCon, POPT_BADOPTION_NOALIAS), poptStrerror (c));

      if (poptPeekArg (optCon) || (identify ? 1 : 0) + (respond ? 1 : 0) + (sign ? 1 : 0) != 1)
      {
         poptPrintUsage (optCon, stderr, 0);
         return -1;
      }
   }
   libusb_device_handle *usb = usb_connect (port);
   if (!usb)
      errx (1, "Failed to open USB %s", port);

   {                            // Card insert...
      card_status_t status = card_status (usb);
      if (status == card_error)
         errx (1, "Card status error");
      if (status == card_missing)
      {
         fprintf (stderr, "Insert card\n");
         while (status == card_missing)
         {
            uint8_t rx[2];
            usb_int_rx (usb, sizeof (rx), rx, 10000);
            status = card_status (usb);
         }
      }
   }

   uint16_t status = 0;
   uint8_t rx[256];
   uint8_t len;

   {                            // Power on
      if ((len = ccid_power_on (usb, voltage, sizeof (rx), rx)) < 2)
         warnx ("Power on fail");
      else if (debug)
         dumphex ("ATR", len, rx);
   }
   {                            // Select file (try two different ones)
      if ((status = select_file (usb, 0x00, 0x04, 0, 7, (uint8_t[])
                                 {
                                 0xA0, 0x00, 0x00, 0x00, 0x03, 0x80, 0x02}
           )) >> 8 != 0x61 && (status = select_file (usb, 0x00, 0x04, 0, 7, (uint8_t[])
                                                     {
                                                     0xA0, 0x00, 0x00, 0x00, 0x04, 0x80, 0x02}
                               )) >> 8 != 0x61)
         warnx ("Select file failed, may be wrong card");
      else
         get_response (usb, status & 0xFF, sizeof (rx), rx);
   }

   {                            // Not sure what this does...
      uint8_t tx[] = { 0x80, 0xA8, 0x00, 0x00, 0x02, 0x83, 0x00 };
      if ((len = card_txn (usb, sizeof (tx), tx, sizeof (rx), rx)) != 2 || *rx != 0x61)
         warnx ("Failed 0xA8");
      else
         get_response (usb, rx[1], sizeof (rx), rx);
   }

   if (pan)
   {                            // Get PAN -- TODO presumably this used to work once upon a time.
      if (!(len = read_file (usb, 0x02, 0x0C, 0, sizeof (rx), rx)) || rx[len - 2] != 0x90)
         errx (1, "Could not read PAN");
      for (int n = 4; n < 12; n++)
         printf ("%02X", rx[n]);
      printf ("\n");

   }

   if (identify || respond || sign || debug)
   {                            // send PIN
      if (pin)
      {                         // Send PIN
         uint8_t tx[] = { 0x00, 0x20, 0x00, 0x80, 0x08, 0x24, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF };
         uint8_t n = 6;
         for (const char *p = pin; *p && isdigit (*p) && n < sizeof (tx); p++)
         {
            tx[n] = ((*p & 0xF) << 4) + 0xF;
            if (p[1] && isdigit (*p))
            {
               p++;
               tx[n] = (tx[n] & 0xF0) + (*p & 0xF);
            }
            n++;
         }
         if (card_txn (usb, sizeof (tx), tx, sizeof (rx), rx) < 2)
            errx (1, "PIN fail");
         if (*rx != 0x90)
         {
            if (*rx == 0x63 && rx[1] >> 4 == 12)
               errx (1, "Wrong PIN, %d tries remaining", rx[1] & 0xF);
            errx (1, "PIN failed");
         }
      } else
      {                         // User reader to get PIN
         // TODO 
      }
   }


   if (identify || respond || sign)
   {                            // OTP logic
      uint8_t tx[36] = { 0x80, 0xAE, 0x80, 0x00, 29 };
      tx[19] = 0x80;
      tx[26] = tx[27] = tx[28] = 1;
      if (respond || sign)
      {                         // Challenge digits or account
         const char *chal = respond ? : sign;
         int n = 0;
         for (int p = 0; chal[p] && chal[p] != '+'; p++)
            if (isdigit (chal[p]))
               n++;
         for (int p = 0; chal[p] && chal[p] != '+' && n; p++)
            if (isdigit (chal[p]))
            {
               n--;
               if (n < 8)
                  tx[33 - n / 2] |= ((chal[p] & 0xF) << ((n & 1) ? 4 : 0));
            }
      }
      if (sign)
      {                         // Amount
         const char *amount = sign;
         while (*amount && *amount != '+')
            amount++;
         if (*amount != '+')
            errx (1, "--sign=ACCOUNT+AMOUNT");
         amount++;
         int n = 0;
         for (int p = 0; amount[p]; p++)
            if (isdigit (amount[p]))
               n++;
         for (int p = 0; amount[p] && n; p++)
            if (isdigit (amount[p]))
            {
               n--;
               if (n < 12)
                  tx[10 - n / 2] |= ((amount[p] & 0xF) << ((n & 1) ? 4 : 0));
            }
      }
      if (card_txn (usb, 5 + tx[4], tx, sizeof (rx), rx) != 2 || *rx != 0x61)
         errx (1, "OTP fail");
      if ((len = get_response (usb, rx[1], sizeof (rx), rx)) != 22 || rx[len - 2] != 0x90)
         errx (1, "OTP fail");
      uint32_t res = ((1 << 25) | (rx[4] << 17) | ((rx[10] & 0x01) << 16) | (rx[11] << 8) | rx[12]);
      printf ("%08lu\n", res);

      // TODO there is a further message that perhaps advances the OTP
   }
   // Power off
   ccid_power_off (usb);

   usb_disconnect (usb);
   poptFreeContext (optCon);
   return 0;
}
