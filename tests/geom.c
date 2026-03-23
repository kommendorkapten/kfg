#include <stdio.h>
#include <math.h>

#include "km_geom.h"
#include "km_phys.h"

int main(void)
{
        int fail = 0;

        struct vec3 v0 = { .a = { -1.0f, 0.0f, -2.0f } };
        struct vec3 v1 = { .a = { -1.0f, 0.0f, 1.0f } };
        struct vec3 v2 = { .a = { 2.0f, 0.0f, 1.0f } };
        struct vec3 e1, e2;
        struct vec3 n;
        struct object o = {0};
        struct mesh m = {0};
        float vn;
        float t;
        float u;
        float v;
        int coll;

        m.restitution = 1.0f;
        o.restitution = 1.0f;

        e1 = vec3_sub(v1, v0);
        e2 = vec3_sub(v2, v0);

        n = vec3_cross(e1, e2);
        n = vec3_norm(n);

        // Test that a particle collides
        o.p.v = (struct vec3){ .a = { 0.0f, -1.0f, 0.0f } };
        o.p.p = (struct vec3){ .a = { 0.0f, 0.5f, 0.0 } };
        coll = ray_tri_intersect(&o.p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                print_particle(&o.p);
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
        o.p.p.y = 1.5f;
        coll = ray_tri_intersect(&o.p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                // As the particle's ray passes through the surface the
                // result coll should be one
                print_particle(&o.p);
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
        o.p.p.y = -0.5f;
        coll = ray_tri_intersect(&o.p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                print_particle(&o.p);
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
        o.p.p.y = 0.1f;
        o.p.v.y = 1.0f;

        coll = ray_tri_intersect(&o.p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                print_particle(&o.p);
                printf("should not have collided\n");
                fail = 1;
        }
        // we are 0.1 units past the surfce, t should reflect this
        if (fabsf(-0.1f - t) > 0.0001)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        // Verify that a particle at the surface will not collide
        o.p.p.y = v0.y;
        o.p.v.y = -0.5f;
        coll = ray_tri_intersect(&o.p, &v0, &v1, &v2, &t, &u, &v);

        if (coll)
        {
                print_particle(&o.p);
                printf("should not have collided\n");
                fail = 1;
        }
        if (t > 0.0001)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        // Verify that the collision response is correct
        o.p.p.y = 0.1f;
        o.p.v.x = 1.0;
        o.p.v.y = -1.0f;
        o.p.v.z = 1.0f;

        vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);
        // with restitution constant of 1.0f, the v.y component should be
        // negated, x and z should be unaffected
        if (fabsf(o.p.v.y - 1.0f) > 0.001)
        {
                printf("unexpected p.v.y %f\n", o.p.v.y);
                fail = 1;
        }
        if (o.p.v.x != 1.0f || o.p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", o.p.v.x, o.p.v.z);
                fail = 1;
        }

        // Particle is now moving away from surface,
        // colide again, should print error message
        printf("Expected error message\n");
        vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);

        o.p.p.y = 0.1f;
        o.p.v.x = 1.0;
        o.p.v.y = -1.0f;
        o.p.v.z = 1.0f;

        vn = vec3_dot(n, o.p.v);
        o.restitution = 0.5f;
        m.restitution = 0.5f;
        collide_object(&m, &o, n, vn);
        // with restitution constant of 0.5f, the v.y component should be
        // negated and halfed, x and z should be unaffected
        if (fabsf(o.p.v.y - 0.5f) > 0.001)
        {
                printf("unexpected p.v.y %f\n", o.p.v.y);
                fail = 1;
        }
        if (o.p.v.x != 1.0f || o.p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", o.p.v.x, o.p.v.z);
                fail = 1;
        }

        if (fail)
        {
                printf("test failed\n");
        }
        else
        {
                printf("test successful\n");
        }

        return fail;
}
