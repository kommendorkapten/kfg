all: free_fall geom test_math test_friction test_phys

include ../common.mk

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

test_friction: test_friction.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

test_phys: test_phys.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

test: free_fall geom test_math test_friction test_phys
	./free_fall
	./geom
	./test_math
	./test_phys
#	./test_friction

clean:
	rm -rf free_fall geom test_math test_friction objs/*
