# PINSentry

This is a command line for bank card validation, as per Barclays PIN Sentry, and Lloyds, and Nationwide, and others. Basically a command line to do what the bank card checking device does.

## Key functions

The key functions are 

- IDENTIFY - provides an 8 digit code to prove you have the card and know the PIN
- RESPOND - allow entry of a code and get a response, with PIN check
- SIGN - allow entry for an account and amount to validate a payment, with PIN check.

## PIN Entry

The command line allows all of the above with, or without `--pin=XXXX`. With, it uses the PIN, but without it uses the PIN entry feature of a CCID Reader PIN PAD.
