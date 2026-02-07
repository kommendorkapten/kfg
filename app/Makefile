include ../common.mk

SDL_CFLAGS  := $(shell sdl2-config --cflags)
SDL_LDFLAGS := $(shell sdl2-config --libs)

FRAMEWORKS = -framework Metal      \
             -framework QuartzCore \
             -framework Foundation \
             -framework CoreGraphics

CFLAGS    += $(SDL_CFLAGS) -I../src
OBJCFLAGS += $(SDL_CFLAGS) -I../src
LDFLAGS    = $(SDL_LDFLAGS) $(FRAMEWORKS)

TARGET = km_app

DEPS = ../src/objs/km_geom.o \
	../src/objs/km_math.o \
	../src/objs/km_phys.o \
	../src/objs/timing.o \
	../src/objs/metal_renderer.o \
        ../src/objs/km_input.o \
	../src/objs/timing.o \
	../src/objs/km_mat4.o \
	../src/objs/km_window.o

all: objs $(TARGET)

objs:
	@mkdir -p objs

$(TARGET): $(OBJS) $(DEPS)
	$(CC) -o $@ $^ $(LDFLAGS)
	cp ../src/objs/shaders.metallib .

clean:
	rm -rf objs $(TARGET) shaders.metallib

.PHONY: all clean
