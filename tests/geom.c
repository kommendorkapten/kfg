#include <stdio.h>
#include <math.h>

#include "km_geom.h"

int main(void)
{
        int fail = 0;

        struct vec3 v0 = { .a = { -1.0f, 0.0f, -2.0f } };
        struct vec3 v1 = { .a = { -1.0f, 0.0f, 1.0f } };
        struct vec3 v2 = { .a = { 2.0f, 0.0f, 1.0f } };
        struct particle p = {0};
        float t;
        float u;
        float v;
        int coll;

        // Test that a particle collides
        p.v = (struct vec3){ .a = { 0.0f, -1.0f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, 0.5f, 0.0 } };
        coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                print_particle(&p);
                printf("should have collided\n");
                fail = 1;
        }
        // Intersection should be at 0.5f
        if (fabsf(0.5f - t) > 0.001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }
        // u & b should be 0.3333
        if (fabsf(0.3333333f - u) > 0.001f)
        {
                printf("unexpected u: %f\n", u);
                fail = 1;
        }

        if (fabsf(0.3333333f - v) > 0.001f)
        {
                printf("unexpected v: %f\n", v);
                fail = 1;
        }

        // Test that a particle won't collide (to far away)
        p.p.y = 1.5f;
        coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                // As the particle's ray passes through the surface the
                // result coll should be one
                print_particle(&p);
                printf("should have collided\n");
                fail = 1;
        }
        // As the velocity is too low, t should be 1.5f (i.e 1.5 steps before
        // it acutally collides
        if (fabsf(1.5f - t) > 0.0001)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        // Test that a particle wont' collide (other side of surface)
        p.p.y = -0.5f;
        coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                print_particle(&p);
                printf("should not have collided\n");
                fail = 1;
        }
        // we are 0.5 units past the surfce, t should reflect this
        if (fabsf(-0.5f - t) > 0.0001)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        // Test that a particle won't collide (close but travel in opposite
        // direction
        p.p.y = 0.1f;
        p.v.y = 1.0f;

        coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                print_particle(&p);
                printf("should not have collided\n");
                fail = 1;
        }
        // we are 0.1 units past the surfce, t should reflect this
        if (fabsf(-0.1f - t) > 0.0001)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        // Verify that the collision response is correct
        p.p.y = 0.1f;
        p.v.x = 1.0;
        p.v.y = -1.0f;
        p.v.z = 1.0f;

        collide_particle(&p, &v0, &v1, &v2, 1.0f);
        // with restitution constant of 1.0f, the v.y component should be
        // negated, x and z should be unaffected
        if (fabsf(p.v.y - 1.0f) > 0.001)
        {
                printf("unexpected p.v.y %f\n", p.v.y);
                fail = 1;
        }
        if (p.v.x != 1.0f || p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", p.v.x, p.v.z);
                fail = 1;
        }

        // Particle is now moving away from surface,
        // colide again, should print error message
        printf("Expected error message\n");
        collide_particle(&p, &v0, &v1, &v2, 1.0f);

        p.p.y = 0.1f;
        p.v.x = 1.0;
        p.v.y = -1.0f;
        p.v.z = 1.0f;

        collide_particle(&p, &v0, &v1, &v2, 0.5f);
        // with restitution constant of 0.5f, the v.y component should be
        // negated and halfed, x and z should be unaffected
        if (fabsf(p.v.y - 0.5f) > 0.001)
        {
                printf("unexpected p.v.y %f\n", p.v.y);
                fail = 1;
        }
        if (p.v.x != 1.0f || p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", p.v.x, p.v.z);
                fail = 1;
        }

        if (fail)
        {
                printf("test failed\n");
        }

        return fail;
}
