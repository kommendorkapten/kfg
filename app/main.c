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

int main(int argc, char *argv[])
{
        struct km_window window;
        struct km_input  input;
        struct renderer  *renderer = NULL;
        int fullscreen = 0;
        int width      = KM_DEFAULT_WIDTH;
        int height     = KM_DEFAULT_HEIGHT;
        Uint64 now, last;
        float dt;
        int i;

        memset(&window, 0, sizeof(window));
        memset(&input,  0, sizeof(input));

        for (i = 1; i < argc; i++)
        {
                if (strcmp(argv[i], "--fullscreen") == 0 ||
                    strcmp(argv[i], "-f") == 0)
                {
                        fullscreen = 1;
                }
                else if (strcmp(argv[i], "--width") == 0 && i + 1 < argc)
                {
                        width = atoi(argv[++i]);
                }
                else if (strcmp(argv[i], "--height") == 0 && i + 1 < argc)
                {
                        height = atoi(argv[++i]);
                }
        }

        if (SDL_Init(SDL_INIT_VIDEO) < 0)
        {
                fprintf(stderr, "SDL_Init: %s\n", SDL_GetError());
                return 1;
        }

        if (km_window_create(&window, "KM",
                             width, height, fullscreen,
                             RENDERER_METAL) != 0)
        {
                SDL_Quit();
                return 1;
        }

        // init renderer
        renderer = metal_renderer_create();
        if (!renderer ||
            renderer->init(renderer, window.sdl_window,
                           width, height) != 0)
        {
                fprintf(stderr, "Failed to initialise renderer\n");
                km_window_destroy(&window);
                SDL_Quit();
                return 1;
        }

        last = SDL_GetPerformanceCounter();

        // go!
        while (!input.quit)
        {
                if (!km_process_input(&input))
                {
                        break;
                }

                if (input.window_resized)
                {
                        renderer->resize(renderer,
                                         input.new_width,
                                         input.new_height);
                }

                now = SDL_GetPerformanceCounter();
                dt  = (float)(now - last)
                    / (float)SDL_GetPerformanceFrequency();
                last = now;

                renderer->render(renderer, dt);
        }

        renderer->cleanup(renderer);
        free(renderer);
        km_window_destroy(&window);
        SDL_Quit();

        return 0;
}
