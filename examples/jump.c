#include <stdio.h>
#include <stdlib.h>
#include <unistd.h>
#include <time.h>
#include <math.h>
#include <SDL.h>
#include "km_scene.h"
#include "km_renderer.h"
#include "km_window.h"
#include "km_input.h"
#include "metal/metal_renderer.h"
#include "km_geom.h"
#include "timing.h"
#include "common.h"

int main(void)
{
        struct scene scene = {0};
        struct km_window window = {0};
        struct km_input  input = {0};
        struct renderer  *renderer = NULL;
        Uint64 now, last;
        int fullscreen = 0;
        float margin = 0.002f; // margin for vsync during sleep
        int slowmo = 1;

        input.width = KM_DEFAULT_WIDTH;
        input.height = KM_DEFAULT_HEIGHT;

        scene_init(&scene);
        default_world(&scene.w, 60);

        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
                fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
                return 1;
        }

        if (km_window_create(&window, "KM",
                             input.width, input.height, fullscreen,
                             RENDERER_METAL) != 0)
        {
                SDL_Quit();
                return 1;
        }

        // init renderer
        renderer = metal_renderer_create();
        if (!renderer ||
            renderer->init(renderer, window.sdl_window,
                           input.width, input.height) != 0)
        {
                fprintf(stderr, "Failed to initialise renderer\n");
                km_window_destroy(&window);
                SDL_Quit();
                return 1;
        }

        // Setup camera
        scene.cam.pos = (struct vec3){ .a = { 10.0f, 10.0f, 25.0f } };
        scene.cam.center = (struct vec3){ .a = { -0.0f, 0.0f, 0.0f } };
        scene.cam.up = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };

        struct vec3 cd = vec3_sub(scene.cam.pos, scene.cam.center);
        float r = sqrtf(vec3_dot(cd, cd));
        input.phi = acosf(cd.y / r);
        input.theta = atan2f(cd.z, cd.x);

        struct mesh* m = gen_mesh(100.0f, 100.0f, 1.0f);
        mesh_translate(m, (struct vec3){ .a = {-50.0f, 0.0f, -50.0f} });
        mesh_colorize(m);
        struct mesh wall;

        init_vert_plane(&wall, 0.0f);

        world_add_mesh(&scene.w, m);
        world_add_mesh(&scene.w, &wall);

        struct entity e;

        memset(&e, 0, sizeof(struct entity));
        e.surfaces = malloc(1 * sizeof(struct mesh));
        e.surface_count = 1;
        e.o.p.p.x = -20.0f;
        e.o.p.p.y = 0.001f;
        e.o.p.v.x = 14.0f;
        e.o.m = 1.0f;
        e.o.m_inv = 1.0f;
        e.o.area = 0.3f;
        e.o.drag_c = 0.47f;
        e.o.restitution = 0.9f;
        e.o.static_mu = 0.15f;
        e.o.dynamic_mu = 0.1f;
        e.o.p.rad = 0.0f; // 0.0f non zero radius breaks
        e.a.speed = 0.8f; // 0.2 rad/sec
        e.animate = &animate_rot_y;
        init_cube(e.surfaces);

        scene_add_entity(&scene, &e);

        if (renderer->upload(renderer,
                             scene.w.surfaces, scene.w.surface_count,
                             scene.entities, scene.entity_count) != 0)
        {
                fprintf(stderr, "Failed to upload meshes\n");
                renderer->cleanup(renderer);
                free(renderer);
                km_window_destroy(&window);
                SDL_Quit();
                return 1;
        }

        int step = 0;
        last = SDL_GetPerformanceCounter();
        while (!input.quit)
        {
                long ns;

                if (!km_process_input(&input, &scene))
                {
                        break;
                }

                if (input.window_resized)
                {
                        renderer->resize(renderer,
                                         input.width,
                                         input.height);
                }

                if (input.az == 'q')
                {
                        input.quit = 1;
                }
                if (input.az == 'p')
                {
                        input.pause = !input.pause;
                }
                now = SDL_GetPerformanceCounter();
                float dt  = (float)(now - last)
                        / (float)SDL_GetPerformanceFrequency();
                last = now;

                // update objects
                if (!input.pause)
                {
                        for (unsigned int i = 0; i < scene.entity_count; i++)
                        {
                                update_object(step, &scene.w, &scene.entities[i]->o);
                        }
                }


                renderer->render(renderer, &scene, dt);

                float elapsed = (float)(SDL_GetPerformanceCounter() - last)
                      / (float)SDL_GetPerformanceFrequency();
                float remaining = scene.w.dt - elapsed - margin;

                if (remaining > 0)
                {
                        ns = (long)(remaining * 1000000000.0f);
                        timing_sleep(ns * slowmo);
                }

                step++;
        }

        renderer->cleanup(renderer);
        free(renderer);

        km_window_destroy(&window);
        SDL_Quit();

        return 0;
}
