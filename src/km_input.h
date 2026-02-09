/*
 * Input handling – keyboard and mouse event processing.
 */

#ifndef KM_INPUT_H
#define KM_INPUT_H

struct scene;

struct km_input
{
        int width;
        int height;
        int mouse_x;
        int mouse_y;
        int mouse_dx;
        int mouse_dy;
        int quit;
        int window_resized;
};

/**
 * Process all pending SDL events.
 *
 * @param input Input state (updated in place).
 * @param scene the scene to modify
 * @return 1 to keep running, 0 when quit has been requested.
 */
extern int km_process_input(struct km_input*, struct scene*);

#endif /* KFG_INPUT_H */
