all: free_fall geom test_math test_friction test_phys

include ../common.mk

CFLAGS += -I../src

DEPS = ../src/objs/km_geom.o \
        ../src/objs/km_math.o \
        ../src/objs/km_phys.o \
        ../src/objs/timing.o \
	../src/objs/km_plat.o \
        ../lib/objs/cJSON.o

free_fall: free_fall.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

geom: geom.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_math: test_math.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_friction: test_friction.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test_phys: test_phys.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

test: free_fall geom test_math test_friction test_phys
	./free_fall
	./geom
	./test_math
	./test_phys
#	./test_friction

clean:
	rm -rf free_fall geom test_math test_friction test_phys objs/*
