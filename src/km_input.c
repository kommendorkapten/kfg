#include <SDL.h>
#include <stdio.h>
#include "km_input.h"

int km_process_input(struct km_input *input)
{
        SDL_Event event;

        input->mouse_dx       = 0;
        input->mouse_dy       = 0;
        input->window_resized = 0;

        while (SDL_PollEvent(&event))
        {
                switch (event.type)
                {
                case SDL_QUIT:
                        input->quit = 1;
                        return 0;

                case SDL_KEYDOWN:
                        switch (event.key.keysym.sym)
                        {
                        case SDLK_ESCAPE:
                                input->quit = 1;
                                return 0;
                        case SDLK_UP:
                                printf("Key pressed: UP\n");
                                break;
                        case SDLK_DOWN:
                                printf("Key pressed: DOWN\n");
                                break;
                        case SDLK_LEFT:
                                printf("Key pressed: LEFT\n");
                                break;
                        case SDLK_RIGHT:
                                printf("Key pressed: RIGHT\n");
                                break;
                        default:
                                break;
                        }
                        break;

                case SDL_MOUSEMOTION:
                {
                        int prev_x = input->mouse_x;
                        int prev_y = input->mouse_y;

                        input->mouse_x  = event.motion.x;
                        input->mouse_y  = event.motion.y;
                        input->mouse_dx = input->mouse_x - prev_x;
                        input->mouse_dy = input->mouse_y - prev_y;

                        printf("Mouse: pos=(%d, %d) delta=(%d, %d)\n",
                               input->mouse_x, input->mouse_y,
                               input->mouse_dx, input->mouse_dy);
                        break;
                }

                case SDL_WINDOWEVENT:
                        if (event.window.event == SDL_WINDOWEVENT_RESIZED)
                        {
                                input->window_resized = 1;
                                input->new_width  = event.window.data1;
                                input->new_height = event.window.data2;
                        }
                        break;

                default:
                        break;
                }
        }

        return 1;
}
