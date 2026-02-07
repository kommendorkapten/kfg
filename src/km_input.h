/*
 * Input handling – keyboard and mouse event processing.
 */

#ifndef KM_INPUT_H
#define KM_INPUT_H

struct km_input
{
        int mouse_x;
        int mouse_y;
        int mouse_dx;
        int mouse_dy;
        int quit;
        int window_resized;
        int new_width;
        int new_height;
};

/**
 * Process all pending SDL events.
 *
 * @param input  Input state (updated in place).
 * @return 1 to keep running, 0 when quit has been requested.
 */
extern int km_process_input(struct km_input *input);

#endif /* KFG_INPUT_H */
