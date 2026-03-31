#include <SDL.h>
#include <stdio.h>
#include <math.h>
#include "km_input.h"
#include "km_scene.h"
#include "km_math.h"

static const float ARROW_MOV = 0.314f;
static const float MOUSE_SCALE = 10.0f;

int km_process_input(struct km_input *input, struct scene* scene)
{
        struct vec3 d;
        SDL_Event event;
        float r;

        input->mouse_dx       = 0;
        input->mouse_dy       = 0;
        input->window_resized = 0;
        input->az             = 0;

        while (SDL_PollEvent(&event))
        {
                switch (event.type)
                {
                case SDL_QUIT:
                        input->quit = 1;
                        return 0;

                case SDL_KEYDOWN:

                        if (event.key.keysym.sym >= SDLK_a &&
                            event.key.keysym.sym <= SDLK_z)
                        {
                                input->az = event.key.keysym.sym;
                                break;
                        }

                        d = vec3_sub(scene->cam.pos,
                                     scene->cam.center);
                        r = sqrtf(vec3_dot(d, d));

                        switch (event.key.keysym.sym)
                        {
                        case SDLK_ESCAPE:
                                input->quit = 1;
                                return 0;
                        case SDLK_UP:
                                input->phi += ARROW_MOV;
                                break;
                        case SDLK_DOWN:
                                input->phi -= ARROW_MOV;
                                break;
                        case SDLK_LEFT:
                                input->theta += ARROW_MOV;
                                break;
                        case SDLK_RIGHT:
                                input->theta -= ARROW_MOV;
                                break;
                        default:
                                break;
                        }

                        // Prevent rotating onto the y-axis as the
                        // camera's up vector is the y-axis.
                        if (input->phi > M_PI)
                        {
                                input->phi = (float)M_PI - 0.001f;
                        }
                        if (input->phi < 0.001)
                        {
                                input->phi = 0.001f;
                        }

                        // Calculate new camera position
                        scene->cam.pos.x = r * sinf(input->phi) *
                                cosf(input->theta);
                        scene->cam.pos.y = r * cosf(input->phi);
                        scene->cam.pos.z = r * sinf(input->phi) *
                                sinf(input->theta);
                        break;
                case SDL_MOUSEMOTION:
                {
                        int prev_x = input->mouse_x;
                        int prev_y = input->mouse_y;

                        input->mouse_x  = event.motion.x;
                        input->mouse_y  = event.motion.y;
                        input->mouse_dx = input->mouse_x - prev_x;
                        input->mouse_dy = input->mouse_y - prev_y;

                        if (event.motion.state & SDL_BUTTON_LMASK)
                        {
                                scene->cam.center.x -= MOUSE_SCALE *
                                        ((float)input->mouse_dx /
                                         (float)input->width);

                                scene->cam.center.y -= MOUSE_SCALE *
                                        ((float)input->mouse_dy /
                                         (float)input->height);
                        }
                        break;
                }

                case SDL_WINDOWEVENT:
                        if (event.window.event == SDL_WINDOWEVENT_RESIZED)
                        {
                                input->window_resized = 1;
                                input->width  = event.window.data1;
                                input->height = event.window.data2;
                        }
                        break;

                default:
                        break;
                }
        }

        return 1;
}
