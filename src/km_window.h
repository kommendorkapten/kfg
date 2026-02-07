/*
 * SDL window management.
 */

#ifndef KM_WINDOW_H
#define KM_WINDOW_H

#include "km_renderer.h"

#define KM_DEFAULT_WIDTH  1024
#define KM_DEFAULT_HEIGHT  768

struct km_window
{
        struct SDL_Window *sdl_window;
        int width;
        int height;
        int fullscreen;
};

/**
 * Create an SDL window.
 *
 * @param w          Window state to initialise.
 * @param title      Window title.
 * @param width      Window width  (screen coordinates).
 * @param height     Window height (screen coordinates).
 * @param fullscreen Non-zero for fullscreen-desktop mode.
 * @param backend    Renderer backend (determines SDL window flags).
 * @return 0 on success, -1 on error.
 */
extern int  km_window_create(struct km_window *w,
                             const char *title,
                             int width,
                             int height,
                             int fullscreen,
                             enum renderer_backend backend);

extern void km_window_destroy(struct km_window *w);

#endif /* KM_WINDOW_H */
