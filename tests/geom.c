#include "km_geom.h"
#include "km_phys.h"
#include "test.h"

static struct vec3 tri_normal(void);
static int test_ray_hit(void);
static int test_ray_far(void);
static int test_ray_other_side(void);
static int test_ray_opposite_dir(void);
static int test_ray_at_surface(void);
static int test_collide_full_restitution(void);
static int test_collide_moving_away(void);
static int test_collide_half_restitution(void);

/* Shared triangle for all geom tests */
static const struct vec3 v0 = { .a = { -1.0f, 0.0f, -2.0f } };
static const struct vec3 v1 = { .a = { -1.0f, 0.0f, 1.0f } };
static const struct vec3 v2 = { .a = { 2.0f, 0.0f, 1.0f } };

static struct vec3 tri_normal(void)
{
        struct vec3 e1 = vec3_sub(v1, v0);
        struct vec3 e2 = vec3_sub(v2, v0);
        return vec3_norm(vec3_cross(e1, e2));
}

static int test_ray_hit(void)
{
        int fail = 0;
        float t, u, v;
        struct particle p = {0};

        p.v = (struct vec3){ .a = { 0.0f, -1.0f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, 0.5f, 0.0f } };
        int coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                printf("should have collided\n");
                fail = 1;
        }
        if (fabsf(0.5f - t) > 0.001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }
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

        return fail;
}

static int test_ray_far(void)
{
        int fail = 0;
        float t, u, v;
        struct particle p = {0};

        p.v = (struct vec3){ .a = { 0.0f, -1.0f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, 1.5f, 0.0f } };
        int coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (!coll)
        {
                printf("should have collided\n");
                fail = 1;
        }
        if (fabsf(1.5f - t) > 0.0001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        return fail;
}

static int test_ray_other_side(void)
{
        int fail = 0;
        float t, u, v;
        struct particle p = {0};

        p.v = (struct vec3){ .a = { 0.0f, -1.0f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, -0.5f, 0.0f } };
        int coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                printf("should not have collided\n");
                fail = 1;
        }
        if (fabsf(-0.5f - t) > 0.0001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        return fail;
}

static int test_ray_opposite_dir(void)
{
        int fail = 0;
        float t, u, v;
        struct particle p = {0};

        p.v = (struct vec3){ .a = { 0.0f, 1.0f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, 0.1f, 0.0f } };
        int coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                printf("should not have collided\n");
                fail = 1;
        }
        if (fabsf(-0.1f - t) > 0.0001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        return fail;
}

static int test_ray_at_surface(void)
{
        int fail = 0;
        float t, u, v;
        struct particle p = {0};

        p.v = (struct vec3){ .a = { 0.0f, -0.5f, 0.0f } };
        p.p = (struct vec3){ .a = { 0.0f, v0.y, 0.0f } };
        int coll = ray_tri_intersect(&p, &v0, &v1, &v2, &t, &u, &v);
        if (coll)
        {
                printf("should not have collided\n");
                fail = 1;
        }
        if (t > 0.0001f)
        {
                printf("unexpected t: %f\n", t);
                fail = 1;
        }

        return fail;
}

static int test_collide_full_restitution(void)
{
        int fail = 0;
        struct vec3 n = tri_normal();
        struct object o = {0};
        struct mesh m = {0};

        m.restitution = 1.0f;
        o.restitution = 1.0f;
        o.p.p.y = 0.1f;
        o.p.v.x = 1.0f;
        o.p.v.y = -1.0f;
        o.p.v.z = 1.0f;

        float vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);
        if (fabsf(o.p.v.y - 1.0f) > 0.001f)
        {
                printf("unexpected p.v.y %f\n", o.p.v.y);
                fail = 1;
        }
        if (o.p.v.x != 1.0f || o.p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", o.p.v.x, o.p.v.z);
                fail = 1;
        }

        return fail;
}

static int test_collide_moving_away(void)
{
        struct vec3 n = tri_normal();
        struct object o = {0};
        struct mesh m = {0};

        m.restitution = 1.0f;
        o.restitution = 1.0f;
        o.p.p.y = 0.1f;
        o.p.v.x = 1.0f;
        o.p.v.y = 1.0f;
        o.p.v.z = 1.0f;

        printf("Expected error message\n");
        float vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);

        return 0;
}

static int test_collide_half_restitution(void)
{
        int fail = 0;
        struct vec3 n = tri_normal();
        struct object o = {0};
        struct mesh m = {0};

        m.restitution = 0.5f;
        o.restitution = 0.5f;
        o.p.p.y = 0.1f;
        o.p.v.x = 1.0f;
        o.p.v.y = -1.0f;
        o.p.v.z = 1.0f;

        float vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);
        if (fabsf(o.p.v.y - 0.5f) > 0.001f)
        {
                printf("unexpected p.v.y %f\n", o.p.v.y);
                fail = 1;
        }
        if (o.p.v.x != 1.0f || o.p.v.z != 1.0f)
        {
                printf("unexpected x %f or z %f\n", o.p.v.x, o.p.v.z);
                fail = 1;
        }

        return fail;
}

static struct test_entry tests[] = {
        {"ray_tri: hit",              test_ray_hit},
        {"ray_tri: far away",         test_ray_far},
        {"ray_tri: other side",       test_ray_other_side},
        {"ray_tri: opposite dir",     test_ray_opposite_dir},
        {"ray_tri: at surface",       test_ray_at_surface},
        {"collide: full restitution", test_collide_full_restitution},
        {"collide: moving away",      test_collide_moving_away},
        {"collide: half restitution", test_collide_half_restitution},
};
RUN_TESTS(tests)
