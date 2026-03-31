include ../common.mk

SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LDFLAGS := $(shell sdl2-config --libs)

FRAMEWORKS = -framework Metal      \
             -framework QuartzCore \
             -framework Foundation \
             -framework CoreGraphics

CFLAGS     += $(SDL_CFLAGS) -I../src
LDFLAGS     = $(SDL_LDFLAGS) $(FRAMEWORKS)
LINT_FLAGS += $(SDL_CFLAGS)

TARGET = gen_mesh

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_input.o \
	../src/objs/km_mat4.o \
	../src/objs/km_window.o \
	../src/objs/metal_renderer.o \
	../src/objs/km_plat.o \
	../lib/objs/cJSON.o

all: objs $(TARGET)

objs:
	@mkdir -p objs

$(TARGET): $(OBJS) $(DEPS)
	$(CC) -o $@ $^ $(LDFLAGS)
	cp ../src/objs/shaders.metallib .

clean:
	rm -rf objs $(TARGET)

.PHONY: all clean
