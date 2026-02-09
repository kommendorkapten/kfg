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

/*
ran for 6016ms
pos: 0.000000 -27.801714 0.000000
vel: 0.000000 -5.837711 0.000000
acl: 0.000000 0.000000 0.000000
for: 0.000000 0.000000 0.000000

bounce
17 seconds
pos: 0.000000 0.000517 0.000000
vel: 0.000000 -0.098088 0.000000
acl: 0.000000 -9.819977 0.000000
18 seconds
19 seconds
20 seconds
21 seconds
22 seconds
23 seconds
24 seconds
ran for 25016ms
pos: 0.000000 0.000176 0.000000
vel: 0.000000 -0.070332 0.000000
acl: 0.000000 -9.820011 0.000000
*/

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
        print_particle(&objs[0].p);

        // create a mesh that (-1,1) (-1,-1) (1, -1) (1,1) in the x-z plane
        struct mesh s1;
        s1.vertices = malloc(4 * sizeof(struct vertex));
        s1.indices = malloc(6 * sizeof(int));
        s1.vertex_count = 4;
        s1.index_count = 6;
        s1.restitution = 1.0f;

        s1.vertices[0].pos = (struct vec3){ .a = {-1.0f, 0.0f, -1.0f} };
        s1.vertices[1].pos = (struct vec3){ .a = {-1.0f, 0.0f, 1.0f} };
        s1.vertices[2].pos = (struct vec3){ .a = {1.0f, 0.0f, 1.0f} };
        s1.vertices[3].pos = (struct vec3){ .a = {1.0f, 0.0f, -1.0f} };

        s1.indices[0] = 0;
        s1.indices[1] = 1;
        s1.indices[2] = 2;
        s1.indices[3] = 2;
        s1.indices[4] = 3;
        s1.indices[5] = 0;

        w.g = (struct vec3){ .a = {0.0f, -9.82f, 0.0f} };
        w.dt = (float)((double)PERIOD/(double)SECOND);
        w.air_density = 1.225f;
        w.surfaces = &s1;
        w.surface_count = 1;
        w.ss_thr   = 0.008f * 0.008f; // 8mm/s
        w.ss_c_thr = 0.08f * 0.08f; // 8cm/s

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

                                printf("%f seconds\n", (step + 1.0f)/ FREQ);
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
}
