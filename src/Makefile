include ../common.mk

SDL_CFLAGS  := $(shell sdl2-config --cflags)
CFLAGS      += -I../src $(SDL_CFLAGS)
OBJCFLAGS   += $(SDL_CFLAGS)
LINT_FLAGS  += $(SDL_CFLAGS)

# Metal shader sources
METAL_SRCS   = $(wildcard metal/shaders/*.metal)
METAL_AIRS   = $(METAL_SRCS:metal/shaders/%.metal=objs/%.air)
METAL_LIB    = objs/shaders.metallib

ifeq ($(UNAME),Darwin)
PLAT_CFLAGS = -D_DARWIN_C_SOURCE
endif

ifeq ($(UNAME),Darwin)
all: objs $(OBJS) objs/metal_renderer.o $(METAL_LIB)
else
all: objs $(OBJS)
endif

objs:
	@mkdir -p objs

objs/km_plat.o: km_plat.c
	$(CC) $(CFLAGS) $(PLAT_CFLAGS) -c -o $@ $<

clean:
	rm -rf objs/*


objs/metal_renderer.o: metal/metal_renderer.m
	$(CC) $(OBJCFLAGS) -DSHADER_LIB_PATH='"shaders.metallib"' -c -o $@ $<

# Compile each .metal source to an intermediate .air file
objs/%.air: metal/shaders/%.metal
	xcrun metal -c $< -o $@

# Link all .air files into one .metallib
$(METAL_LIB): $(METAL_AIRS)
	xcrun metallib $^ -o $@
