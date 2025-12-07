
pinsentry:   pinsentry.c
	gcc -O -o $@ $< -lpopt -lm -g -lusb-1.0
