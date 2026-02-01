CFLAGS=-D_POSIX_C_SOURCE=200809L -Wall -W -pedantic -std=c99

all: md1 bounce

md1: md1.c
	cc $(CFLAGS) -o $@ $^

bounce: bounce.c timing.o
	cc $(CFLAGS) -o $@ $^

%.o: %.c
	cc $(CFLAGS) -c -o $@ $^

clean:
	rm -f md1
