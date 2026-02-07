include ../common.mk

TARGET =
CFLAGS += -I../src $(shell sdl2-config --cflags)
OBJCFLAGS += $(shell sdl2-config --cflags)

# Metal shader sources
METAL_SRCS   = $(wildcard metal/shaders/*.metal)
METAL_AIRS   = $(METAL_SRCS:metal/shaders/%.metal=objs/%.air)
METAL_LIB    = objs/shaders.metallib

all: $(OBJS) objs/metal_renderer.o $(METAL_LIB)

objs/metal_renderer.o: metal/metal_renderer.m
	$(CC) $(OBJCFLAGS) -DSHADER_LIB_PATH='"shaders.metallib"' -c -o $@ $<

# Compile each .metal source to an intermediate .air file
objs/%.air: metal/shaders/%.metal
	xcrun metal -c $< -o $@

# Link all .air files into one .metallib
$(METAL_LIB): $(METAL_AIRS)
	xcrun metallib $^ -o $@

clean:
	rm -rf objs/*
