/*
 * KFG – SDL + Metal demo application.
 *
 * Usage:
 *   ./kfg_app                          (1024×768 windowed)
 *   ./kfg_app --fullscreen             (fullscreen desktop)
 *   ./kfg_app --width 1280 --height 720
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <SDL.h>

#include "km_renderer.h"
#include "km_window.h"
#include "km_input.h"
#include "metal/metal_renderer.h"
#include "km_phys.h"
#include "km_scene.h"
#include "km_geom.h"
#include "timing.h"

int main(int argc, char *argv[])
{
        struct scene scene = {0};
        struct km_window window = {0};
        struct km_input  input = {0};
        struct renderer  *renderer = NULL;
        int fullscreen = 0;
        int verbose    = 0;
        const char* world_file = "mesh.json";
        const char* water_file = "water.json";
        Uint64 now, last;
        float margin = 0.002f; // margin for vsync during sleep
        int slowmo = 1;

        input.width = KM_DEFAULT_WIDTH;
        input.height = KM_DEFAULT_HEIGHT;
        default_world(&scene.w, 60);

        for (int i = 1; i < argc; i++)
        {
                if (strcmp(argv[i], "--fullscreen") == 0 ||
                    strcmp(argv[i], "-f") == 0)
                {
                        fullscreen = 1;
                }
                else if (strcmp(argv[i], "-v") == 0)
                {
                        verbose = 1;
                }
                else if (strcmp(argv[i], "-w") == 0 && i + 1 < argc)
                {
                        world_file = argv[++i];
                }
                else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc)
                {
                        input.width = atoi(argv[++i]);
                }
                else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc)
                {
                        input.height = atoi(argv[++i]);
                }
        }

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
        scene.cam.pos = (struct vec3){ .a = { 0.0f, 8.4f, 30.0f } };
        scene.cam.center = (struct vec3){ .a = { 0.0f, 0.0f, 0.0f } };
        scene.cam.up = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };
        struct vec3 cd = vec3_sub(scene.cam.pos, scene.cam.center);
        float r = sqrtf(vec3_dot(cd, cd));
        input.phi = acosf(cd.y / r);
        input.theta = atan2f(cd.z, cd.x);

        // Setup static meshes
        scene.w.surfaces = load_meshes(world_file, &scene.w.surface_count);
        if (scene.w.surfaces)
        {
                struct vec3 level = (struct vec3){ .a = {0.0f, -1.0f, 0.0f } };
                mesh_translate(scene.w.surfaces, &level);
        }

        scene.w.waters = load_meshes(water_file, &scene.w.water_count);

        struct water w;
        init_water(&w, scene.w.waters, scene.w.surfaces);

        mesh_colorize_water(scene.w.waters);
        mesh_normalize(scene.w.waters);

        scene.entity_count = 0;
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

        // Create GPU buffers for dynamic water meshes
        if (renderer->update(renderer,
                             scene.w.waters, scene.w.water_count, 1) != 0)
        {
                fprintf(stderr, "Failed to create dynamic meshes\n");
                renderer->cleanup(renderer);
                free(renderer);
                km_window_destroy(&window);
                SDL_Quit();
                return 1;
        }

        last = SDL_GetPerformanceCounter();
        // go!
        int counter = 0;
        while (!input.quit)
        {
                int print_step = 0;
                long ns = 0;

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

                now = SDL_GetPerformanceCounter();
                float dt  = (float)(now - last)
                        / (float)SDL_GetPerformanceFrequency();
                dt = dt / (float)slowmo;
                last = now;

                // update objects
                for (int i = 0; i < scene.entity_count; i++)
                {
                        update_object(1, &scene.w, &scene.entities[i].o);
                }

                // animate
                for (int i = 0; i < scene.entity_count; i++)
                {
                        if (scene.entities[i].animate)
                        {
                                scene.entities[i].animate(&scene.entities[i], dt);
                        }
                }

                // Animate environment
                for (int i = 0; i < scene.w.water_count; i++)
                {
                        update_water(&w, scene.w.waters + i, dt);
                        mesh_normalize(scene.w.waters);
                        mesh_colorize_water(scene.w.waters);
                }

                // Update dynamic mesh GPU data
                renderer->update(renderer,
                                 scene.w.waters, scene.w.water_count, 0);

                renderer->render(renderer, &scene, dt);
                float elapsed = (float)(SDL_GetPerformanceCounter() - last)
                      / (float)SDL_GetPerformanceFrequency();
                float remaining = scene.w.dt - elapsed - margin;


                counter++;

                if (counter == 100)
                {
                        counter = 0;
                        print_step = verbose;
                }

                if (remaining > 0)
                {
                        ns = (long)(remaining * 1000000000.0f);
                        timing_sleep(ns * slowmo);
                }

                if (print_step)
                {
                        printf("dt: %fs\n", dt);
                        printf("remaining: %fs\n", remaining);
                        printf("ns %ldns\n", ns);
                        printf("camera pos: %f %f %f\n", scene.cam.pos.x, scene.cam.pos.y, scene.cam.pos.x);
                        printf("camera center: %f %f %f\n", scene.cam.center.x, scene.cam.center.y, scene.cam.center.x);
                        printf("camera up: %f %f %f\n", scene.cam.up.x, scene.cam.up.y, scene.cam.up.x);
                }
        }

        renderer->cleanup(renderer);
        free(renderer);
        for (int i = 0; i < scene.w.surface_count; i++)
        {
                mesh_free(scene.w.surfaces + i);
        }
        free(scene.w.surfaces);

        km_window_destroy(&window);
        SDL_Quit();

        return 0;
}
