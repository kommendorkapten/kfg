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

void init_cube(struct mesh* m);
void init_plane(struct mesh* m, float w, float h);

int main(int argc, char *argv[])
{
        struct scene scene = {0};
        struct km_window window = {0};
        struct km_input  input = {0};
        struct renderer  *renderer = NULL;
        int fullscreen = 0;
        int verbose    = 0;
        Uint64 now, last;
        float margin = 0.002f; // margin for vsync during sleep

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
        scene.cam.pos = (struct vec3){ .a = { 0.0f, 0.4f, 15.0f } };
        scene.cam.center = (struct vec3){ .a = { 0.0f, 0.0f, 0.0f } };
        scene.cam.up = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };

        // Setup static meshes
        scene.w.surfaces = malloc(1 * sizeof(struct mesh));
        scene.w.surface_count = 1;
        init_plane(scene.w.surfaces, 10.0f, 10.0f);

        // Setup dynamic meshes
        scene.entities = malloc(1 * sizeof(struct entity));
        scene.entity_count = 1;

        // Entity 1
        memset(scene.entities, 0, sizeof(struct entity));
        scene.entities[0].surfaces = malloc(1 * sizeof(struct mesh));
        scene.entities[0].surface_count = 1;
        scene.entities[0].o.p.p.y = 3.0f;
        scene.entities[0].o.m = 1.0f;
        scene.entities[0].o.area = 0.3f;
        scene.entities[0].o.drag_c = 0.47f;
        scene.entities[0].o.restitution = 0.9f;
        scene.entities[0].o.p.rad = 1.0f;

        init_cube(&scene.entities[0].surfaces[0]);

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
                last = now;

                // update objects
                for (int i = 0; i < scene.entity_count; i++)
                {
                        update_object(1, &scene.w, &scene.entities[i].o);
                }

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
                        timing_sleep(ns);
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

void init_cube(struct mesh* m)
{
        m->vertex_count = 24;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 36;
        m->indices = malloc(m->index_count * sizeof(int));
        m->restitution = 1.0f;

        /* Front face  (z = +1)  normal ( 0,  0,  1) */
        m->vertices[ 0] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 1] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 2] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 3] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = { 0,  0,  1} } };
        /* Back face   (z = -1)  normal ( 0,  0, -1) */
        m->vertices[ 4] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 5] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 6] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 7] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 0,  0, -1} } };
        /* Top face    (y = +1)  normal ( 0,  1,  0) */
        m->vertices[ 8] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[ 9] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[10] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[11] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = { 0,  1,  0} } };
        /* Bottom face (y = -1)  normal ( 0, -1,  0) */
        m->vertices[12] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[13] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[14] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[15] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = { 0, -1,  0} } };
        /* Right face  (x = +1)  normal ( 1,  0,  0) */
        m->vertices[16] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[17] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[18] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[19] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 1,  0,  0} } };
        /* Left face   (x = -1)  normal (-1,  0,  0) */
        m->vertices[20] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[21] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[22] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[23] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = {-1,  0,  0} } };

        /* Front  */
        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 2;
        m->indices[3] = 0; m->indices[4] = 2; m->indices[5] = 3;

        /* Back   */
        m->indices[6] = 4; m->indices[7] = 5; m->indices[8] = 6;
        m->indices[9] = 4; m->indices[10] = 6; m->indices[11] = 7;

        /* Top    */
        m->indices[12] = 8; m->indices[13] = 9; m->indices[14] = 10;
        m->indices[15] = 8; m->indices[16] = 10; m->indices[17] = 11;

       /* Bottom */
        m->indices[18] = 12; m->indices[19] = 13; m->indices[20] = 14;
        m->indices[21] = 12; m->indices[22] = 15; m->indices[23] = 15;

       /* Right  */
        m->indices[24] = 16; m->indices[25] = 17; m->indices[26] = 18;
        m->indices[27] = 16; m->indices[28] = 18; m->indices[29] = 19;

        /* Left   */
        m->indices[30] = 20; m->indices[31] = 21; m->indices[32] = 22;
        m->indices[33] = 20; m->indices[34] = 22; m->indices[35] = 23;
}

void init_plane(struct mesh* m, float w, float h)
{
        m->vertex_count = 4;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 6;
        m->indices = malloc(m->index_count * sizeof(int));
        m->restitution = 0.7f;

        w = w / 2.0f;
        h = h / 2.0f;
        float tilt = -2.0f;

        m->vertices[0] = (struct vertex){ .pos = { .a = {-w,  tilt, -h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[1] = (struct vertex){ .pos = { .a = {-w,  tilt,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[2] = (struct vertex){ .pos = { .a = { w,  0,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[3] = (struct vertex){ .pos = { .a = { w,  0, -h} }, .normal = { .a = { 0,  1,  0} } };

        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 3;
        m->indices[3] = 1; m->indices[4] = 2; m->indices[5] = 3;
}
