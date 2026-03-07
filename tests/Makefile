include ../common.mk

all: free_fall geom test_math

CFLAGS += -I../src

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o \
	../lib/objs/cJSON.o

free_fall: free_fall.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

geom: geom.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

test_math: test_math.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

test: free_fall geom test_math
	./free_fall
	./geom
	./test_math

clean:
	rm -rf free_fall geom test_math objs/*
