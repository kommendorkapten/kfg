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

void init_cube(struct mesh* m);
void init_plane(struct mesh* m, float w, float h, int tilt);
void init_vert_plane(struct mesh* m, float x);

int main(int argc, char* argv[])
{
        struct scene scene = {0};
        struct km_window window = {0};
        struct km_input  input = {0};
        struct renderer  *renderer = NULL;
        Uint64 now, last;
        int fullscreen = 0;
        int tilt = 0;
        int opt;
        float margin = 0.002f; // margin for vsync during sleep
        int slowmo = 1;

        while ((opt = getopt(argc, argv, "t")) != -1)
        {
                switch (opt)
                {
                case 't':
                        tilt = 1;
                        break;
                default:
                        fprintf(stderr, "usage: %s [-w width] [-h height] [-d delta] [-o output] [-f file]\n",
                                argv[0]);
                        return 1;
                }
        }

        input.width = KM_DEFAULT_WIDTH;
        input.height = KM_DEFAULT_HEIGHT;
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
        scene.cam.pos = (struct vec3){ .a = { -5.0f, 1.0f, 15.0f } };
        scene.cam.center = (struct vec3){ .a = { -5.0f, 0.0f, 0.0f } };
        scene.cam.up = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };

        struct vec3 cd = vec3_sub(scene.cam.pos, scene.cam.center);
        float r = sqrtf(vec3_dot(cd, cd));
        input.phi = acosf(cd.y / r);
        input.theta = atan2f(cd.z, cd.x);

        scene.w.surface_count = 3;
        scene.w.surfaces = malloc((unsigned int)scene.w.surface_count *
                                  sizeof(struct mesh));
        init_plane(&scene.w.surfaces[0], 10.0f, 10.0f, tilt);
        init_plane(&scene.w.surfaces[1], 20.0f, 10.0f, 0);
        for (int i = 0; i < scene.w.surfaces[1].vertex_count; i++)
        {
                scene.w.surfaces[1].vertices[i].pos.x -= 5.0f;
                scene.w.surfaces[1].vertices[i].pos.y = -2.0f;
        }
        init_vert_plane(&scene.w.surfaces[2], -10.0f);

        scene.entities = malloc(1 * sizeof(struct entity));
        scene.entity_count = 1;
        memset(scene.entities, 0, sizeof(struct entity));
        scene.entities[0].surfaces = malloc(1 * sizeof(struct mesh));
        scene.entities[0].surface_count = 1;
        scene.entities[0].o.p.p.y = 3.0f;
        scene.entities[0].o.m = 1.0f;
        scene.entities[0].o.m_inv = 1.0f;
        scene.entities[0].o.area = 0.3f;
        scene.entities[0].o.drag_c = 0.47f;
        scene.entities[0].o.restitution = 0.9f;
        scene.entities[0].o.static_mu = 0.15f;
        scene.entities[0].o.dynamic_mu = 0.1f;
        scene.entities[0].o.p.rad = 0.0f; // 0.0f non zero radius breaks
        scene.entities[0].a.speed = 0.8f; // 0.2 rad/sec
        scene.entities[0].animate = &animate_rot_y;
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
                        for (int i = 0; i < scene.entity_count; i++)
                        {
                                update_object(step, &scene.w, &scene.entities[i].o);
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

                if ((step % 100) == 0 && 0)
                {
                        printf("--- Print particle ------\n");
                        print_particle(&scene.entities->o.p);
                }
        }

        renderer->cleanup(renderer);
        free(renderer);

        km_window_destroy(&window);
        SDL_Quit();

        return 0;
}


void init_cube(struct mesh* m)
{
        m->vertex_count = 24;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 36;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->restitution = 1.0f;
        m->static_mu = 0.7f;
        m->dynamic_mu = 0.45f;

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
        m->indices[21] = 12; m->indices[22] = 14; m->indices[23] = 15;

       /* Right  */
        m->indices[24] = 16; m->indices[25] = 17; m->indices[26] = 18;
        m->indices[27] = 16; m->indices[28] = 18; m->indices[29] = 19;

        /* Left   */
        m->indices[30] = 20; m->indices[31] = 21; m->indices[32] = 22;
        m->indices[33] = 20; m->indices[34] = 22; m->indices[35] = 23;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }
}

void init_plane(struct mesh* m, float w, float h, int tilt)
{
        float y = tilt ? -1.0f : 0.0f;

        m->vertex_count = 4;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 6;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->inward_normals = malloc(m->index_count * sizeof(struct vec3));
        m->restitution = 0.7f;
        m->static_mu = 0.15f;
        m->dynamic_mu = 0.1f;

        w = w / 2.0f;
        h = h / 2.0f;

        m->vertices[0] = (struct vertex){ .pos = { .a = {-w,  y, -h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[1] = (struct vertex){ .pos = { .a = {-w,  y,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[2] = (struct vertex){ .pos = { .a = { w,  0,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[3] = (struct vertex){ .pos = { .a = { w,  0, -h} }, .normal = { .a = { 0,  1,  0} } };

        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 3;
        m->indices[3] = 1; m->indices[4] = 2; m->indices[5] = 3;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }

        mesh_normalize(m);
        mesh_inward_normalize(m);
}

void init_vert_plane(struct mesh* m, float x)
{
        m->vertex_count = 4;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 6;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->inward_normals = malloc(m->index_count * sizeof(struct vec3));
        m->restitution = 0.7f;
        m->static_mu = 0.15f;
        m->dynamic_mu = 0.1f;

        float w = 5.0f;
        float h = 5.0f;

        m->vertices[0] = (struct vertex){ .pos = { .a = {x,  h, -w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[1] = (struct vertex){ .pos = { .a = {x,  h,  w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[2] = (struct vertex){ .pos = { .a = {x,  -h,  w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[3] = (struct vertex){ .pos = { .a = {x,  -h, -w} }, .normal = { .a = { 1, 0, 0} } };

        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 3;
        m->indices[3] = 1; m->indices[4] = 2; m->indices[5] = 3;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }

        mesh_normalize(m);
        mesh_inward_normalize(m);
}
