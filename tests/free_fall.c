#include <stdio.h>
#include <math.h>

#include "km_phys.h"
#include "km_geom.h"

#define NUM_OBJS 1

void init_object(struct object* o)
{
        o->p.p = (struct vec3){0.0f, 0.0f, 0.0f};
        o->p.v = (struct vec3){0.0f, 0.0f, 0.0f};
        o->p.a = (struct vec3){0.0f, 0.0f, 0.0f};
        o->steady_state = 0;
}

int free_fall(int, int);

int main(void)
{
        int fail = 0;
        // Only run shorter simulations. The accumulated error when using
        // float builds up over time (that is, the number of steps).

        // 1 minute, 60hz
        fail = fail || free_fall(60, 60);
        // 1 minute, 100hz
        fail = fail || free_fall(60, 100);
        // 1 hour, 60hz
        fail = fail || free_fall(3600, 60);
        // 1 hour, 100hz
        fail = fail || free_fall(3600, 100);
        // 2 hours, 60hz
        fail = fail || free_fall(7200, 60);
        // 2 hours at 100 hz is too much, the error rate gets close to 0.5%

        if (fail)
        {
                printf("test failed\n");
        }
        return fail;
}

int free_fall(int duration, int freq)
{
        struct world w;
        struct object o = {0};
        int steps;
        int ret = 0;

        // Free fall, no drag and no area
        o.p.p.y = 0.0f;
        o.m = 1.0f;
        o.area = 0.0f;
        o.drag_c = 0.0;

        w.g = (struct vec3){0.0f, -9.82f, 0.0f};
        w.dt = 1.0f / (float)freq;
        w.air_density = 1.225f;
        w.num_surfaces = 0;

        steps = (int)((1.0f / w.dt) * (float)duration + 0.5f);

        for (int i = 0; i < steps; i++)
        {
                update_object(i, &w, &o);
        }

        // v = v0 + at
        float exp_v = o.p.a.y * (float)duration;
        // d = vo*t + 0.5 * a * t^2
        float exp_d = 0.5f * o.p.a.y * (float)(duration * duration);
        float err_v = fabsf((exp_v - o.p.v.y) / exp_v);
        float err_d = fabsf((exp_d - o.p.p.y) / exp_d);

        // tolerate 0.3% error / hour
        float err_thr = 0.003f;
        int h = duration / 3600;
        if (h > 1)
        {
                err_thr *= (float)h;
        }
        if (err_v > err_thr)
        {
                printf("duration %ds frequency %dhz\n", duration, freq);
                printf("Too high error for velocity: %f > %f\n",
                       err_v, err_thr);
                ret = 1;
        }
        if (err_d > err_thr)
        {
                printf("duration %ds frequency %dhz\n", duration, freq);
                printf("Too high error for distance: %f > %f\n",
                       err_d, err_thr);
                ret = 1;
        }

        return ret;
}
