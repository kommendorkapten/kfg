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
struct scene;
struct mesh;
struct entity;

enum renderer_backend
{
        RENDERER_METAL  = 0,
        RENDERER_VULKAN = 1
};

struct renderer {
        int  (*init)(struct renderer* r, struct SDL_Window* window,
                     int w, int h);
        int  (*upload)(struct renderer* r,
                       struct mesh* const* static_meshes,
                       unsigned int static_count,
                       struct entity* const* entities,
                       unsigned int entity_count);
        int  (*update)(struct renderer* r,
                       struct mesh* const* meshes,
                       unsigned int count, int create);
        void (*render)(struct renderer* r, struct scene* s, float dt);
        void (*resize)(struct renderer* r, int width, int height);
        void (*cleanup)(struct renderer* r);
        void *ctx;
};

#endif /* KM_RENDERER_H */
