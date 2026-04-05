TARGETS=bounce

include ../common.mk

ifeq ($(UNAME),Darwin)
  TARGETS += visb jump
endif

all: $(TARGETS)

DEPS = objs/common.o \
	../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o \
	../src/objs/km_input.o \
	../src/objs/km_window.o \
	../src/objs/km_mat4.o \
	../src/objs/km_scene.o \
	../src/objs/km_plat.o \
	../lib/objs/cJSON.o

ifeq ($(UNAME),Darwin)
  DEPS += ../src/objs/metal_renderer.o
endif

SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LDFLAGS := $(shell sdl2-config --libs)

ifeq ($(UNAME),Darwin)
  FRAMEWORKS = -framework Metal      \
               -framework QuartzCore \
               -framework Foundation \
               -framework CoreGraphics
endif

CFLAGS     += $(SDL_CFLAGS) -I../src
LDFLAGS    += $(SDL_LDFLAGS) $(FRAMEWORKS)
LINT_FLAGS += $(SDL_CFLAGS)

objs:
	@mkdir -p objs

bounce: bounce.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)

visb: visb.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	cp ../src/objs/shaders.metallib .

jump: jump.c $(DEPS)
	$(CC) $(CFLAGS) -o $@ $^ $(LDFLAGS)
	cp ../src/objs/shaders.metallib .

clean:
	rm -rf $(TARGETS) objs/*
