#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "km_phys.h"
#include "km_geom.h"

#define NUM_OBJS 1

int free_fall(int, int);
int with_drag(int, int, float);
int upwards(int);
int bounce(int);
int test_drag(void);
int test_coll(void);

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

        // Run with drag force (expected values computed numerically)
        fail = fail || with_drag(5, 60, 45.251652f);
        fail = fail || with_drag(5, 100, 45.251652f);
        fail = fail || with_drag(10, 60, 98.561859f);
        fail = fail || with_drag(10, 100, 98.561859f);

        fail = fail || upwards(60);
        fail = fail || upwards(100);
        fail = fail || bounce(60);
        fail = fail || test_drag();
        fail = fail || test_coll();

        if (fail)
        {
                printf("test failed\n");
        }
        else
        {
                printf("test passed\n");
        }
        return fail;
}

/*
 * Test pure free fall (no drag) using Verlet
 * integration. Compares simulated velocity and
 * position against the analytical solutions
 * v = a*t and d = 0.5*a*t^2.
 * Tolerance scales with simulated duration
 * (0.3% per hour).
 */
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

/*
 * Test free fall with air drag. Verifies that
 * the simulated velocity converges to the
 * analytical terminal velocity
 *   v_t = -sqrt(2mg / (rho * Cd * A))
 * and that the final position matches a
 * numerically precomputed reference value.
 */
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

/*
 * Test an object launched upward with drag.
 * Verifies energy is not gained: when the
 * object returns to y=0 its downward speed
 * must not exceed the initial launch speed,
 * as drag dissipates energy on both the
 * ascent and descent.
 */
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

        f = vec3_sub(f, drag_force(&w, &o));

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

/*
 * Test that a slowly rising object with drag
 * does not gain altitude over several steps.
 * Starts near the ground with a small upward
 * velocity and verifies the net displacement
 * is non-positive, catching energy-injection
 * bugs in the integrator.
 */
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

        f = vec3_sub(f, drag_force(&w, &o));

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
                printf("travelled too far %f @%dhz\n", dp, freq);
        }

        return ret;
}

/*
 * Test drag_force in isolation for various
 * velocity vectors. The expected drag is
 *   F = -0.5 * rho * Cd * A * |v| * v
 * Each case verifies that the computed force
 * matches the analytical value and opposes
 * the velocity direction.
 */
int test_drag(void)
{
        struct world w;
        struct object o = {0};
        struct vec3 f;
        int ret = 0;
        float thr = 1e-5f;

        // rho=1.225, area=0.3, Cd=0.47
        // half_rho * area * Cd = 0.086_3625
        float hr_cd = 0.5f * KM_PHYS_AIR_DENS * 0.3f * 0.47f;

        w.air_density = KM_PHYS_AIR_DENS;
        w.surface_count = 0;
        o.m = 1.0f;
        o.area = 0.3f;
        o.drag_c = 0.47f;

        struct {
                struct vec3 vel;
                const char* label;
        } cases[] = {
                // axis-aligned, positive
                {{ .a = { 1.0f,  0.0f,  0.0f}}, "+x"},
                {{ .a = { 0.0f,  1.0f,  0.0f}}, "+y"},
                {{ .a = { 0.0f,  0.0f,  1.0f}}, "+z"},
                // axis-aligned, negative
                {{ .a = {-1.0f,  0.0f,  0.0f}}, "-x"},
                {{ .a = { 0.0f, -1.0f,  0.0f}}, "-y"},
                {{ .a = { 0.0f,  0.0f, -1.0f}}, "-z"},
                // larger magnitudes
                {{ .a = { 5.0f,  0.0f,  0.0f}}, "+5x"},
                {{ .a = { 0.0f, -5.0f,  0.0f}}, "-5y"},
                // two-component combinations
                {{ .a = { 1.0f,  1.0f,  0.0f}}, "+x+y"},
                {{ .a = {-1.0f,  1.0f,  0.0f}}, "-x+y"},
                {{ .a = { 0.0f,  2.0f, -3.0f}}, "+2y-3z"},
                {{ .a = {-2.0f,  0.0f,  4.0f}}, "-2x+4z"},
                // three-component combinations
                {{ .a = { 1.0f,  1.0f,  1.0f}}, "+x+y+z"},
                {{ .a = {-1.0f, -1.0f, -1.0f}}, "-x-y-z"},
                {{ .a = { 3.0f, -4.0f,  0.0f}}, "+3x-4y"},
                {{ .a = {-2.0f,  3.0f, -1.0f}}, "mix"},
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 v = cases[i].vel;
                float speed = sqrtf(
                        v.x * v.x +
                        v.y * v.y +
                        v.z * v.z);
                float k = hr_cd * speed;
                float ex = -k * v.x;
                float ey = -k * v.y;
                float ez = -k * v.z;

                o.p.v = v;
                f = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
                f = vec3_sub(f, drag_force(&w, &o));

                float dx = fabsf(f.x - ex);
                float dy = fabsf(f.y - ey);
                float dz = fabsf(f.z - ez);

                if (dx > thr || dy > thr || dz > thr)
                {
                        printf("drag %s: "
                               "got (%.6f,%.6f,%.6f) "
                               "exp (%.6f,%.6f,%.6f)\n",
                               cases[i].label,
                               f.x, f.y, f.z,
                               ex, ey, ez);
                        ret = 1;
                }

                // verify drag opposes velocity
                float dot =
                        f.x * v.x +
                        f.y * v.y +
                        f.z * v.z;
                if (dot > thr)
                {
                        printf("drag %s: "
                               "not opposing vel "
                               "(dot=%.6f)\n",
                               cases[i].label, dot);
                        ret = 1;
                }
        }

        // zero velocity must produce zero drag
        o.p.v = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        f = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        f = vec3_sub(f, drag_force(&w, &o));
        if (fabsf(f.x) > thr ||
            fabsf(f.y) > thr ||
            fabsf(f.z) > thr)
        {
                printf("drag zero: "
                       "got (%.6f,%.6f,%.6f) "
                       "exp (0,0,0)\n",
                       f.x, f.y, f.z);
                ret = 1;
        }

        // drag must accumulate onto existing force
        o.p.v = (struct vec3){ .a = {1.0f, 0.0f, 0.0f} };
        f = (struct vec3){ .a = {10.0f, 5.0f, -3.0f} };
        f = vec3_sub(f, drag_force(&w, &o));
        float ex_x = 10.0f - hr_cd * 1.0f;
        if (fabsf(f.x - ex_x) > thr ||
            fabsf(f.y - 5.0f) > thr ||
            fabsf(f.z - (-3.0f)) > thr)
        {
                printf("drag accum: "
                       "got (%.6f,%.6f,%.6f) "
                       "exp (%.6f,5.0,-3.0)\n",
                       f.x, f.y, f.z, ex_x);
                ret = 1;
        }

        return ret;
}

int test_coll(void)
{
        struct world w;
        struct mesh m;
        struct object o = {0};
        float epsilon = 0.0001f;
        int ret = 0;

        default_world(&w, 60);

        m.vertex_count = 3;
        m.vertices = malloc(m.vertex_count * sizeof(struct vertex));
        m.vertices[0].pos = (struct vec3) { .a = { -1.0f, 0.0f, -2.0f } };
        m.vertices[1].pos = (struct vec3) { .a = { -1.0f, 0.0f, 1.0f } };
        m.vertices[2].pos = (struct vec3) { .a = { 2.0f, 0.0f, 1.0f } };
        m.index_count = 3;
        m.indices = malloc(m.index_count * sizeof(uint16_t));
        m.indices[0] = 0;
        m.indices[1] = 1;
        m.indices[2] = 2;
        m.restitution = 0.5f;
        m.static_mu = 0.5f;
        m.dynamic_mu = 0.5f;
        w.surfaces = &m;
        w.surface_count = 1;

        o.p.v.y = -1.0f;
        o.p.p.y = 0.02f;
        o.p.a.y = - KM_PHYS_G;
        o.m = 1.0f;
        o.area = 0.3f;
        o.drag_c = 0.0f;
        o.restitution = 0.5;
        o.static_mu = 0.5;
        o.dynamic_mu = 0.5;

        struct {
                float v;
                float y;
                float a;
        } cases[] = {
                // computed by hand
                {-1.1636f, 0.0019697f, -KM_PHYS_G},
                // collision happens at step 2, object bounces up
                // time of collision is at 0.094
                {0.4298f, 0.00873f, -KM_PHYS_G}
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                update_object(i, &w, &o);

                if (fabsf(o.p.v.y - cases[i].v) > epsilon)
                {
                        printf("%d got v %f expected %f d %f\n", i,
                               o.p.v.y, cases[i].v, fabsf(o.p.v.y - cases[i].v));
                        ret = 1;
                }
                if (fabsf(o.p.p.y - cases[i].y) > epsilon)
                {
                        printf("%d got y %f expected %f d %f\n", i,
                               o.p.p.y, cases[i].y, fabsf(o.p.p.y - cases[i].y));
                        ret = 1;
                }
                if (fabsf(o.p.a.y - cases[i].a) > epsilon)
                {
                        printf("%d got a %f expected %f d %f\n", i,
                               o.p.a.y, cases[i].a, fabsf(o.p.a.y - cases[i].a));
                        ret = 1;
                }
        }

        return ret;
}
