TESTS = free_fall geom test_math test_friction test_phys
RUN_TESTS = free_fall geom test_math test_phys test_friction

all: $(TESTS)

include ../common.mk

CFLAGS += -I../src -I.

DEPS = ../src/objs/km_geom.o \
        ../src/objs/km_math.o \
        ../src/objs/km_phys.o \
        ../src/objs/timing.o \
	../src/objs/km_plat.o \
        ../lib/objs/cJSON.o

%: %.c $(DEPS) test.h
	$(CC) $(CFLAGS) -o $@ $< $(DEPS) $(LDFLAGS)

test: $(RUN_TESTS)
	@fail=0; \
	for t in $(RUN_TESTS); do \
		echo "=== $$t ==="; \
		./$$t || fail=1; \
		echo; \
	done; \
	exit $$fail

clean:
	rm -rf $(TESTS) objs/*
