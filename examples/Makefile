all: bounce visb

include ../common.mk

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o \
	../src/objs/km_input.o \
	../src/objs/km_window.o \
	../src/objs/metal_renderer.o \
	../src/objs/km_mat4.o \
	../src/objs/km_scene.o \
	../lib/objs/cJSON.o

SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LDFLAGS := $(shell sdl2-config --libs)

FRAMEWORKS = -framework Metal      \
             -framework QuartzCore \
             -framework Foundation \
             -framework CoreGraphics

CFLAGS    += $(SDL_CFLAGS) -I../src
LDFLAGS    = $(SDL_LDFLAGS) $(FRAMEWORKS)
LINT_FLAGS += $(SDL_CFLAGS)

bounce: bounce.c $(DEPS)
	$(CC) $(CFLAGS) $(LDFLAGS) -o $@ $^ $(LIBS)

visb: visb.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	cp ../src/objs/shaders.metallib .

clean:
	rm -rf bounce objs/*
