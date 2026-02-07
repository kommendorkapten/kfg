/*
 * Abstract renderer interface.
 *
 * Backend-specific implementations (Metal, Vulkan, etc.) populate
 * the function pointers in struct renderer.  The application code
 * only ever calls through these pointers, keeping the rendering
 * backend swappable.
 */

#ifndef KM_RENDERER_H
#define KM_RENDERER_H

struct SDL_Window;

enum renderer_backend
{
        RENDERER_METAL  = 0,
        RENDERER_VULKAN = 1
};

struct renderer {
        int  (*init)(struct renderer *r, struct SDL_Window *window,
                     int w, int h);
        void (*render)(struct renderer *r, float dt);
        void (*resize)(struct renderer *r, int width, int height);
        void (*cleanup)(struct renderer *r);
        void *ctx;
};

#endif /* KM_RENDERER_H */
