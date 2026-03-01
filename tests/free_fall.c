#include <stdio.h>
#include <math.h>

#include "km_phys.h"
#include "km_geom.h"

#define NUM_OBJS 1

int free_fall(int, int);
int with_drag(int, int, float);
int upwards(int);
int bounce(int);

int main(void)
{
        int fail = 0;
        // Only run shorter simulations. The accumulated error when using
        // float builds up over time (that is, the number of steps).
#if 0
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

        // Run with drag force (expected values computed numerically)
        fail = fail || with_drag(5, 60, 45.251652f);
        fail = fail || with_drag(5, 100, 45.251652f);
        fail = fail || with_drag(10, 60, 98.561859f);
        fail = fail || with_drag(10, 100, 98.561859f);

        fail = fail || upwards(60);
        fail = fail || upwards(100);
#endif
        fail = fail || bounce(60);
//        fail = fail || bounce(100);

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

        w.g = (struct vec3){ .a = { 0.0f, -KM_PHYS_G, 0.0f} };
        w.dt = 1.0f / (float)freq;
        w.air_density = KM_PHYS_AIR_DENS;
        w.surface_count = 0;

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

int with_drag(int duration, int freq, float exp_p)
{
        struct world w;
        struct object o = {0};
        int steps;
        int ret = 0;

        o.p.p.y = 0.0f;
        o.m = 1.0f;
        o.area = 0.3f;
        o.drag_c = 0.47f;

        w.g = (struct vec3){ .a = { 0.0f, -KM_PHYS_G, 0.0f } };
        w.dt = 1.0f / (float)freq;
        w.air_density = KM_PHYS_AIR_DENS;
        w.surface_count = 0;

        steps = (int)((1.0f / w.dt) * (float)duration + 0.5f);

        for (int i = 0; i < steps; i++)
        {
                update_object(i, &w, &o);
        }

        float vel_t = - sqrtf((2 * o.m * KM_PHYS_G) /
                            (w.air_density * o.drag_c * o.area));

        float err_thr = 0.003f;
        float err_v = fabsf(vel_t - o.p.v.y) / vel_t;
        if (err_v > err_thr)
        {
                printf("duration %ds frequency %dhz\n", duration, freq);
                printf("Too high error for terminal velocity: %f > %f\n",
                       err_v, err_thr);
                printf("expected terminal velocity %f got %f d: %f\n",
                       vel_t, o.p.v.y, fabsf(vel_t - o.p.v.y));
                ret = 1;
        }

        float err_p = fabsf(fabsf(o.p.p.y) - exp_p) / exp_p;
        if (err_p > err_thr)
        {
                printf("duration %ds frequency %dhz\n", duration, freq);
                printf("Too high error for position: %f > %f\n",
                       err_p, err_thr);
                printf("expected position %f got %f d: %f\n",
                       exp_p, o.p.p.y, fabsf(o.p.p.y) - exp_p);
                ret = 1;
        }

        return ret;
}

int upwards(int freq)
{
        struct world w;
        struct object o = {0};
        struct vec3 f;
        int step;
        int ret = 0;
        float vstart = 0.726153f;

        o.p.p.y = 0.0f;
        o.m = 1.0f;
        o.area = 0.3f;
        o.drag_c = 0.47f;

        w.g = (struct vec3){ .a = { 0.0f, -KM_PHYS_G, 0.0f } };
        w.dt = 1.0f / (float)freq;
        w.air_density = KM_PHYS_AIR_DENS;
        w.surface_count = 0;

        // init force
        f.x = o.m * w.g.x;
        f.y = o.m * w.g.y;
        f.z = o.m * w.g.z;

        drag_force(&f, &w, &o);

        // compute new accelerations
        o.p.a.x = f.x / o.m;
        o.p.a.y = f.y / o.m;
        o.p.a.z = f.z / o.m;

        // init velocity
        o.p.v.y = vstart;

        for (step = 0; ; step++)
        {
                update_object(step, &w, &o);

                if (o.p.p.y + w.dt * o.p.v.y < 0.0f)
                {
                        if (o.p.v.y > fabsf(vstart))
                        {
                                ret = 1;
                        }

                        break;
                }
        }

        return ret;
}

int bounce(int freq)
{
        struct world w;
        struct object o = {0};
        struct vec3 f;
        int step = 0;
        int ret = 0;
        float vstart = 0.247399f;
        float pstart = -1.999002f;

        o.p.p.y = 0.0f;
        o.m = 1.0f;
        o.area = 0.3f;
        o.drag_c = 0.47f;

        w.g = (struct vec3){ .a = { 0.0f, -KM_PHYS_G, 0.0f } };
        w.dt = 1.0f / (float)freq;
        w.air_density = KM_PHYS_AIR_DENS;
        w.surface_count = 0;

        // init force
        f.x = o.m * w.g.x;
        f.y = o.m * w.g.y;
        f.z = o.m * w.g.z;

        drag_force(&f, &w, &o);

        // init acceleration
        o.p.a.x = 0.0f;
        o.p.a.y = -9.827365f;
        o.p.a.z = 0.0f;

        // init pos
        o.p.p.x = 0.0f;
        o.p.p.y = pstart;
        o.p.p.z = 0.0f;

        // init velocity
        o.p.v.x = 0.0f;
        o.p.v.y = vstart;
        o.p.v.z = 0.0f;

        update_object(step, &w, &o);
        update_object(step, &w, &o);
        update_object(step, &w, &o);
        update_object(step, &w, &o);

        float dp = o.p.p.y - pstart;

        if (dp > 0)
        {
                ret = 1;
                printf("travelled too far\n");
        }

        return ret;
}
