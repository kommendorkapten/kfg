#include <SDL.h>
#include <stdio.h>
#include "km_window.h"

int km_window_create(struct km_window *w,
                     const char *title,
                     int width, int height,
                     int fullscreen,
                     enum renderer_backend backend)
{
        Uint32 flags = SDL_WINDOW_SHOWN
                     | SDL_WINDOW_RESIZABLE
                     | SDL_WINDOW_ALLOW_HIGHDPI;

        if (fullscreen)
                flags |= SDL_WINDOW_FULLSCREEN_DESKTOP;

        switch (backend) {
        case RENDERER_METAL:
                flags |= SDL_WINDOW_METAL;
                break;
        case RENDERER_VULKAN:
                flags |= SDL_WINDOW_VULKAN;
                break;
        }

        w->sdl_window = SDL_CreateWindow(title,
                                         SDL_WINDOWPOS_CENTERED,
                                         SDL_WINDOWPOS_CENTERED,
                                         width, height, flags);
        if (!w->sdl_window)
        {
                fprintf(stderr, "Failed to create window: %s\n",
                        SDL_GetError());
                return -1;
        }

        w->width      = width;
        w->height     = height;
        w->fullscreen = fullscreen;

        return 0;
}

void km_window_destroy(struct km_window *w)
{
        if (w->sdl_window)
        {
                SDL_DestroyWindow(w->sdl_window);
                w->sdl_window = NULL;
        }
}
