include ../common.mk

all: bounce

CFLAGS += -I../src

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o

bounce: bounce.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf bounce objs/*
