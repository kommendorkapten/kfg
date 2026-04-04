#include <unistd.h>
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
static int test_gen_mesh(void);
static int test_gen_mesh_large(void);
static int test_point_on_mesh(void);
static int test_write_parse_mesh(void);

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
        int ret = 0;

        m.restitution = 1.0f;
        o.restitution = 1.0f;
        o.p.p.y = 0.1f;
        o.p.v.x = 1.0f;
        o.p.v.y = 1.0f;
        o.p.v.z = 1.0f;

        printf("Expected error message\n");
        float vn = vec3_dot(n, o.p.v);
        collide_object(&m, &o, n, vn);

        // collide_object returns early, velocity
        // should be unaffected
        if (o.p.v.x != 1.0f)
        {
                printf("x velocity modified\n");
                ret = 1;
        }

        if (o.p.v.y != 1.0f)
        {
                printf("y velocity modified\n");
                ret = 1;
        }

        if (o.p.v.z != 1.0f)
        {
                printf("z velocity modified\n");
                ret = 1;
        }

        return ret;
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
        ASSERT_FE(1.0f, o.p.v.x);
        ASSERT_FE(1.0f, o.p.v.z);

        return fail;
}

static int test_gen_mesh(void)
{
        int ret = 0;
        struct mesh* m = gen_mesh(1.0f, 1.0f, 1.0f);
        // length of the inward normal's component when not axis aligned
        float el = sqrtf(2.0f) / 2.0f;

        // Should get two triangles
        ASSERT_IE(4, m->vertex_count);
        ASSERT_IE(6, m->index_count);

        // All vertex normals should be 0, 1, 0
        for (int i = 0; i < m->vertex_count; i++)
        {
                struct vec3 n = {.a = { 0.0f, 1.0f, 0.0f } };
                ASSERT_IE(1, vec3_approx(n, m->vertices[i].normal, F_THR));
        }


        // check inward normals
        struct {
                struct vec3 in;
        } cases[] = {
                // First triangle
                {{ .a = { 1.0f,  0.0f,  0.0f}}},
                {{ .a = {-el,    0.0f, -el}}},
                {{ .a = { 0.0f,  0.0f,  1.0f}}},
                // Second triangle
                {{ .a = { el,    0.0f,  el}}},
                {{ .a = { 0.0f,  0.0f, -1.0f}}},
                {{ .a = {-1.0f,  0.0f,  0.0f}}},
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                if (!vec3_approx(cases[i].in, m->inward_normals[i], F_THR))
                {
                        printf("Wrong inward normal: %d\n", i);
                        vec3_print(cases[i].in);
                        vec3_print(m->inward_normals[i]);
                        ret = 1;
                }
        }

        // Now stretch the mesh and recalculate all normals and
        // make sure the normals are |1|
        m->vertices[1].pos.y = 2.0f;
        m->vertices[3].pos.y = -1.0f;

        mesh_normalize(m);
        mesh_inward_normalize(m);

        for (unsigned int i = 0; i < m->vertex_count; i++)
        {
                ASSERT_FE(1.0f, vec3_dot(m->vertices[i].normal,
                                         m->vertices[i].normal));
        }

        for (unsigned int i = 0; i < m->index_count; i++)
        {
                ASSERT_FE(1.0f, vec3_dot(m->inward_normals[i],
                                         m->inward_normals[i]));
        }

        mesh_free(m);
        free(m);

        return ret;
}

static int test_gen_mesh_large(void)
{
        struct mesh* m = gen_mesh(100.0f, 100.0f, 1.0f);

        ASSERT_IE(101, m->grid_x);
        ASSERT_IE(101, m->grid_z);
        ASSERT_IE(101 * 101, m->vertex_count);
        // 100 x 100 sqares, each square is two triangles.
        // Each triangle has three indices
        ASSERT_IE(100 * 100 * 2 * 3, m->index_count);

        ASSERT_FE(m->vertices[100].pos.x, 100.0f);
        ASSERT_FE(m->vertices[100].pos.y, 0.0f);
        ASSERT_FE(m->vertices[100].pos.z, 0.0f);

        ASSERT_FE(m->vertices[101].pos.x, 0.0f);
        ASSERT_FE(m->vertices[101].pos.y, 0.0f);
        ASSERT_FE(m->vertices[101].pos.z, 1.0f);

        struct vertex* v0;
        struct vertex* v1;
        struct vertex* v2;
        struct vec3 e1;
        struct vec3 e2;
        struct vec3 n;
        struct vec3 up = {.a = {0.0f, 1.0f, 0.0f}};

        mesh_get_tri(&v0, &v1, &v2, m, 2);

        ASSERT_FE(v1->pos.x, 1.0f);
        ASSERT_FE(v1->pos.y, 0.0f);
        ASSERT_FE(v1->pos.z, 1.0f);

        e1 = vec3_sub(v1->pos, v0->pos);
        e2 = vec3_sub(v2->pos, v0->pos);

        ASSERT_FE(e1.x, 0.0f);
        ASSERT_FE(e1.y, 0.0f);
        ASSERT_FE(e1.z, 1.0f);

        n = vec3_cross(e1, e2);
        ASSERT_IE(1, vec3_approx(n, up, F_THR));

        mesh_free(m);
        free(m);

        return 0;
}

static int test_point_on_mesh(void)
{
        struct mesh* m = gen_mesh(1.0f, 1.0f, 1.0f);
        struct vec3 p = {0};

        mesh_translate(m, (struct vec3){.a = {0.0f, 1.0f, 0.0f}});

        // Point below mesh
        ASSERT_IE(0, point_on_mesh(m, p));

        // Point on mesh
        p.y = 1.0f;
        ASSERT_IE(1, point_on_mesh(m, p));

        // Point just above mesh
        p.y = 1.001f;
        ASSERT_IE(1, point_on_mesh(m, p));

        // Point just beside mesh
        p.x = 1.001f;
        ASSERT_IE(0, point_on_mesh(m, p));

        // Point in the middle of the mesh (first triangle)
        p.x = 0.2f;
        p.z = 0.2f;
        ASSERT_IE(1, point_on_mesh(m, p));

        // Point in the middle of the mesh (second triangle)
        p.x = 0.7f;
        p.z = 0.7f;
        ASSERT_IE(1, point_on_mesh(m, p));

        // Point outside
        p.x = -0.1f;
        p.z = -0.1f;
        ASSERT_IE(0, point_on_mesh(m, p));

        mesh_free(m);
        free(m);

        return 0;
}

static int test_write_parse_mesh(void)
{
        struct mesh* m = gen_mesh(2.0f, 2.0f, 1.0f);
        if (!m)
        {
                printf("gen_mesh failed\n");
                return 1;
        }

        m->restitution = 0.42f;
        m->static_mu = 0.33f;
        m->dynamic_mu = 0.21f;

        // Perturb some vertices so normals are non-trivial
        m->vertices[1].pos.y = 0.5f;
        m->vertices[4].pos.y = -0.3f;
        mesh_normalize(m);
        mesh_inward_normalize(m);

        // Write to temp file
        char path[] = "/tmp/kfg_test_XXXXXX";
        int fd = mkstemp(path);
        if (fd < 0)
        {
                printf("mkstemp failed\n");
                mesh_free(m);
                free(m);
                return 1;
        }
        close(fd);

        if (write_meshes(path, m, 1) != 0)
        {
                printf("write_meshes failed\n");
                unlink(path);
                mesh_free(m);
                free(m);
                return 1;
        }

        // Read back
        int count = 0;
        struct mesh* loaded = load_meshes(path, &count);
        unlink(path);

        if (!loaded || count != 1)
        {
                printf("load_meshes failed, count=%d\n", count);
                mesh_free(m);
                free(m);
                return 1;
        }

        int ret = 0;
        struct mesh* r = &loaded[0];

        // Scalar properties
        ASSERT_FE(m->restitution, r->restitution);
        ASSERT_FE(m->static_mu, r->static_mu);
        ASSERT_FE(m->dynamic_mu, r->dynamic_mu);
        ASSERT_IE(m->vertex_count, r->vertex_count);
        ASSERT_IE(m->index_count, r->index_count);
        ASSERT_IE(m->grid_x, r->grid_x);
        ASSERT_IE(m->grid_z, r->grid_z);

        // Vertex positions and colors
        for (int i = 0; i < m->vertex_count; i++)
        {
                if (!vec3_approx(m->vertices[i].pos,
                                 r->vertices[i].pos, F_THR))
                {
                        printf("vertex %d pos mismatch\n", i);
                        ret = 1;
                }
                struct vec4 mc = m->vertices[i].color;
                struct vec4 rc = r->vertices[i].color;
                if (!float_approx(mc.x, rc.x, F_THR) ||
                    !float_approx(mc.y, rc.y, F_THR) ||
                    !float_approx(mc.z, rc.z, F_THR) ||
                    !float_approx(mc.w, rc.w, F_THR))
                {
                        printf("vertex %d color mismatch\n", i);
                        ret = 1;
                }
        }

        // Indices
        for (unsigned int i = 0; i < m->index_count; i++)
        {
                ASSERT_IE(m->indices[i], r->indices[i]);
        }

        // Normals are recomputed from geometry, verify they match
        for (unsigned int i = 0; i < m->vertex_count; i++)
        {
                if (!vec3_approx(m->vertices[i].normal,
                                 r->vertices[i].normal, F_THR))
                {
                        printf("vertex %d normal mismatch\n", i);
                        ret = 1;
                }
        }

        // Inward normals
        for (unsigned int i = 0; i < m->index_count; i++)
        {
                if (!vec3_approx(m->inward_normals[i],
                                 r->inward_normals[i], F_THR))
                {
                        printf("inward normal %d mismatch\n", i);
                        ret = 1;
                }
        }

        mesh_free(m);
        free(m);
        for (int i = 0; i < count; i++)
        {
                mesh_free(&loaded[i]);
        }
        free(loaded);

        return ret;
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
        {"gen_mesh",                  test_gen_mesh},
        {"gen_large_mesh",            test_gen_mesh_large},
        {"point_on_mesh",             test_point_on_mesh},
        {"write_parse_mesh",          test_write_parse_mesh}
};
RUN_TESTS(tests)
