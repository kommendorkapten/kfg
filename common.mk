CC = cc
CFLAGS = -D_POSIX_C_SOURCE=200809L \
         -Wall -W -Wextra -Wconversion -Wsign-conversion \
         -pedantic -O2 \
         -std=c11
OBJCFLAGS = -D_POSIX_C_SOURCE=200809L \
         -Wall -W -Wextra -Wconversion -Wsign-conversion \
         -pedantic -O2 \
         -fobjc-arc

LDFLAGS =
LIBS =

ifdef DEBUG
    CFLAGS += -g -DDEBUG
    OBJCFLAGS += -g -DDEBUG
endif

SRCS ?= $(wildcard *.c)
OBJS = $(SRCS:%.c=objs/%.o)

objs/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: all clean
