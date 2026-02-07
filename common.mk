CC = cc
CFLAGS = -D_POSIX_C_SOURCE=200809L \
         -Wall -W -Wextra -Wconversion -Wsign-conversion \
         -std=c99 -pedantic  -O2
LDFLAGS =
LIBS =

ifdef DEBUG
    CFLAGS += -g -DDEBUG
endif

SRCS ?= $(wildcard *.c)
OBJS = $(SRCS:%.c=objs/%.o)

FOOO = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o

objs/%.o: %.c
	$(CC) $(CFLAGS) -c -o $@ $<

.PHONY: all clean
