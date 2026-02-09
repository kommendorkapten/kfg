include ../common.mk

all: free_fall geom

CFLAGS += -I../src

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o

free_fall: free_fall.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

geom: geom.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

clean:
	rm -rf free_fall geom objs/*
