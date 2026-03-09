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

int main(int argc, char* argv[])
{
        struct scene scene = {0};
        struct km_window window = {0};
        struct km_input  input = {0};
        struct renderer  *renderer = NULL;
        Uint64 now, last;
        int fullscreen = 0;
        float w = 30.0f;
        float h = 30.0f;
        float d = 0.5f;
        const char* output = "mesh.json";
        const char* input_file = NULL;
        int num_peaks = 5;
        int height = 1.0f;
        int radius = 10.0f;
        int opt;

        srand((unsigned)time(NULL));

        while ((opt = getopt(argc, argv, "w:h:d:o:f:")) != -1)
        {
                switch (opt)
                {
                case 'w':
                        w = (float)atof(optarg);
                        break;
                case 'h':
                        h = (float)atof(optarg);
                        break;
                case 'd':
                        d = (float)atof(optarg);
                        break;
                case 'o':
                        output = optarg;
                        break;
                case 'f':
                        input_file = optarg;
                        break;
                default:
                        fprintf(stderr, "usage: %s [-w width] [-h height] [-d delta] [-o output] [-f file]\n",
                                argv[0]);
                        return 1;
                }
        }

        input.width = KM_DEFAULT_WIDTH;
        input.height = KM_DEFAULT_HEIGHT;
        struct vec3 sv = (struct vec3){ .a = { -w/2.0f, 0.0f, -h/2.0f} };

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
        scene.cam.pos = (struct vec3){ .a = { 0.0f, 10.0f, 30.0f } };
        scene.cam.center = (struct vec3){ .a = { 0.0f, 0.0f, 0.0f } };
        scene.cam.up = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };
        struct vec3 cd = vec3_sub(scene.cam.pos, scene.cam.center);
        float r = sqrtf(vec3_dot(cd, cd));
        input.phi = acosf(cd.y / r);
        input.theta = atan2f(cd.z, cd.x);

        struct mesh* m;
        int loaded_from_file = 0;

        if (input_file)
        {
                int count = 0;
                m = load_meshes(input_file, &count);
                if (!m || count == 0)
                {
                        fprintf(stderr, "failed to load mesh from %s\n",
                                input_file);
                        return 1;
                }
                loaded_from_file = 1;
                printf("loaded mesh from %s\n", input_file);
        }
        else
        {
                m = gen_mesh(w, h, d);
                if (!m)
                {
                        fprintf(stderr, "failed to generate mesh\n");
                        return 1;
                }

                mesh_translate(m, &sv);
                mesh_heightmap(m, num_peaks, height, radius);
                mesh_colorize(m);
                mesh_normalize(m);
        }

        printf("vertices: (%d x %d) %d\n",
               m->grid_x,
               m->grid_z,
               m->vertex_count);
        printf("indices:  %d\n", m->index_count);

        if (renderer->update(renderer,
                             m, 1, 1) != 0)
        {
                fprintf(stderr, "Failed to create dynamic meshes\n");
                renderer->cleanup(renderer);
                free(renderer);
                km_window_destroy(&window);
                SDL_Quit();
                return 1;
        }

        last = SDL_GetPerformanceCounter();
        while (!input.quit)
        {
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

                if (input.az == 'n' && !loaded_from_file)
                {
                        mesh_free(m);
                        free(m);

                        m = gen_mesh(w, h, d);
                        if (!m)
                        {
                                fprintf(stderr, "failed to generate mesh\n");
                                return 1;
                        }

                        mesh_translate(m, &sv);
                        mesh_heightmap(m, num_peaks, height, radius);
                        mesh_colorize(m);
                        mesh_normalize(m);

                        if (renderer->update(renderer,
                                             m, 1, 0) != 0)
                        {
                                fprintf(stderr, "Failed to create dynamic meshes\n");
                                renderer->cleanup(renderer);
                                free(renderer);
                                km_window_destroy(&window);
                                SDL_Quit();
                                return 1;
                        }
                }

                if (input.az == 'q')
                {
                        input.quit = 1;
                }
                now = SDL_GetPerformanceCounter();
                float dt  = (float)(now - last)
                        / (float)SDL_GetPerformanceFrequency();
                last = now;

                renderer->render(renderer, &scene, dt);
        }

        renderer->cleanup(renderer);
        free(renderer);
        if (!loaded_from_file)
        {
                if (write_meshes(output, m, 1) != 0)
                {
                        fprintf(stderr, "failed to write mesh to %s\n", output);
                        mesh_free(m);
                        free(m);
                        return 1;
                }
                printf("wrote mesh to %s\n", output);
        }

        mesh_free(m);
        free(m);

        km_window_destroy(&window);
        SDL_Quit();


        return 0;
}
