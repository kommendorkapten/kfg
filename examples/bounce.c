#include <stdio.h>
#include <sched.h>
#include <pthread.h>
#include <string.h>
#include "km_phys.h"
#include "km_geom.h"
#include "km_math.h"
#include "timing.h"

#define PERIOD   16666666
#define FREQ           60
#define SECOND 1000000000
#define NUM_OBJS 1

void set_high_priority(void);
void init_object(struct object*);

int main(int argc, char** argv)
{
        struct timing start;
        struct world w;
        struct object objs[NUM_OBJS];
        long t1;
        int run = 1;
        int step = 0;
        int stop = 25 * FREQ;
        int debug = 1;

        (void)argc;
        (void)argv;

        init_object(&objs[0]);
        objs[0].p.p.y = 5.0f;
        objs[0].m = 1.0f;
        objs[0].area = 0.3f;
        objs[0].drag_c = 0.47f;
        objs[0].restitution = 0.9f;
        objs[0].dynamic_mu = 0.1f;
        objs[0].m_inv = 1.0f;
        print_particle(&objs[0].p);

        // create a 2x2 mesh centered at origin
        struct mesh* s1 = gen_mesh(2.0f, 2.0f, 1.0f);
        mesh_translate(s1, (struct vec3){ .a = {-1.0f, 0.0f, -1.0f} });
        s1->restitution = 0.6f;
        s1->dynamic_mu = 0.5f;

        default_world(&w, FREQ);
        world_add_mesh(&w, s1);

        if (debug)
        {
                printf("Using dt %f\n", w.dt);
        }

        set_high_priority();

        timing_start(&start);
        t1 = timing_current_millis();
        while (run)
        {
                long begin = timing_current_usec();
                char print = 0;

                if (debug)
                {
                        long now = timing_current_millis();

                        if (now - t1 > 1000)
                        {
                                t1 = now;

                                printf("%f seconds\n", ((float)step + 1.0f)/ FREQ);
                                print = 1;
                        }
                }
                update_objects(step, &w, objs, NUM_OBJS, print);

                // wait for next step
                long sbegin = timing_current_usec();
                timing_sleep(PERIOD - (sbegin - begin) * 1000);

                if (step >= stop) {
                        run = 0;
                }
                step++;
        }

        printf("ran for %ldms\n", timing_dur_msec(&start));

        print_particle(&objs[0].p);

        return 0;
}

void set_high_priority(void) {
        struct sched_param param = {
                .sched_priority = sched_get_priority_max(SCHED_FIFO)
        };
        int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (ret)
        {
                printf("failed to set scheduler: %s\n",
                       strerror(ret));
        }
}

void init_object(struct object* o)
{
        o->p.p = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        o->p.v = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        o->p.a = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        o->steady_state = 0;
        o->contact_mesh = NULL;
}
