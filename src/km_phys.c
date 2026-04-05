/*
* Copyright (C) 2026 Fredrik Skogman, skogman - at - gmail.com.
*
* The contents of this file are subject to the terms of the Common
* Development and Distribution License (the "License"). You may not use this
* file except in compliance with the License. You can obtain a copy of the
* License at http://opensource.org/licenses/CDDL-1.0. See the License for the
* specific language governing permissions and limitations under the License.
* When distributing the software, include this License Header Notice in each
* file and include the License file at http://opensource.org/licenses/CDDL-1.0.
*/

#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
#include <assert.h>
#include "km_phys.h"
#include "km_math.h"
#include "km_geom.h"

// Clamp ratio, if the collision is close to head on, the
// impulse force gives a lot of impulse damping in the
// tangential velocity, Coulomb friction is often considered
// lower than the dynamic friction so adjust for that.
#define CCR 0.7f

void print_particle(const struct particle* p)
{
        printf("pos: %f %f %f\n", p->p.x, p->p.y, p->p.z);
        printf("vel: %f %f %f\n", p->v.x, p->v.y, p->v.z);
        printf("acl: %f %f %f\n", p->a.x, p->a.y, p->a.z);
}

void object_set_m(struct object* o, float m)
{
        assert(m > 0.0f);

        o->m = m;
        o->m_inv = 1.0f / m;
}

void default_world(struct world* w, int fps)
{
        memset(w, 0, sizeof(*w));

        w->surface_cap = 128;
        w->surface_count = 0;
        w->surfaces = malloc(w->surface_cap * sizeof(struct mesh*));

        w->water_cap = 128;
        w->water_count = 0;
        w->waters = malloc(w->water_cap * sizeof(struct mesh*));

        w->g = (struct vec3){ .a = {0.0f, -KM_PHYS_G, 0.0f} };
        w->dt = 1.0f / (float)fps;
        w->air_density = KM_PHYS_AIR_DENS;
        w->ss_thr   = 0.008f * 0.008f; // 8mm/s
}

void world_add_mesh(struct world* w, struct mesh* m)
{
        if (w->surface_count == w->surface_cap)
        {
                w->surface_cap <<= 1;
                struct mesh** new = malloc(w->surface_cap *
                                           sizeof(struct mesh*));
                memcpy(new, w->surfaces,
                       w->surface_count * sizeof(struct mesh*));
                free(w->surfaces);
                w->surfaces = new;
        }
        w->surfaces[w->surface_count++] = m;
}

void world_add_water(struct world* w, struct mesh* m)
{
        if (w->water_count == w->water_cap)
        {
                w->water_cap <<= 1;
                struct mesh** new = malloc(w->water_cap *
                                           sizeof(struct mesh*));
                memcpy(new, w->waters,
                       w->water_count * sizeof(struct mesh*));
                free(w->waters);
                w->waters = new;
        }
        w->waters[w->water_count++] = m;
}

void update_objects(int step,
                    const struct world* w,
                    struct object* objs,
                    int n,
                    char print)
{
        for (int i = 0; i < n; i++)
        {
                struct object* o = objs + i;

                update_object(step, w, o);

                if (print)
                {
                        print_particle(&o->p);
                }
        }
}

void update_object(int step, const struct world* w, struct object* o)
{
        float remaining = w->dt;
        int max_iter = 5;

        (void)step;

        assert(o->m_inv > 0.0f);

        if (o->steady_state)
        {
                return;
        }

        while (remaining > 0.0f && max_iter-- > 0)
        {
                struct particle p = {0};
                struct collision toi;
                float v_normal;
                int coll = 0;

                // use current pos and the tentative displacement
                p.p = o->p.p;
                p.v.x = (o->p.v.x + o->p.a.x * remaining * 0.5f) * remaining;
                p.v.y = (o->p.v.y + o->p.a.y * remaining * 0.5f) * remaining;
                p.v.z = (o->p.v.z + o->p.a.z * remaining * 0.5f) * remaining;

                coll = compute_toi(&toi, &p, w->surfaces, w->surface_count);

                // t is time to impact, measured in this step's displacement
                if (!coll || toi.t > 1)
                {
                        // No collision
                        // Check if the object is on a surface
                        if (o->contact_mesh)
                        {
                                if (!point_on_mesh(o->contact_mesh,
                                                   o->p.p))
                                {
                                        // Object slide off
                                        o->contact_mesh = NULL;
                                }
                        }

                        vverlet_step(w, o, remaining);
                        break;
                }

                // Advance particle to collision point
                vverlet_step(w, o, toi.t * remaining);

                // Move the object away from the surface
                // a tiny bit (1mm)
                struct vec3 ns = vec3_scalarm(toi.n, 0.001f);
                o->p.p = vec3_add(o->p.p, ns);

                // Collide the particle. This will mirror
                // the velocity in the collision normal's
                // direction and apply the restitution damping
                v_normal = vec3_dot(o->p.v, toi.n);
                collide_object(toi.m,
                               o,
                               toi.n,
                               v_normal);

                // Recompute v_normal after collision
                v_normal = vec3_dot(o->p.v, toi.n);

                // Check for contact manifold
                // If the bounce height is negligible
                // the apex is when when 0.5 m v^2 = mgh
                // (kinetic energy equals potential energy)
                float bounce_h = (v_normal * v_normal) /
                        (2.0f * KM_PHYS_G);
                if (bounce_h < 0.002f)
                {
                        struct vec3 tmp;

                        // less than 2mm, set the velocity in the normal's
                        // direction to zero
                        tmp = vec3_scalarm(toi.n, v_normal);
                        o->p.v = vec3_sub(o->p.v, tmp);

                        // clamp object to mesh
                        o->contact_mesh = toi.m;
                        o->contact_normal = toi.n;
                        // TODO: update compute toi to ignore the mesh
                        // the particle is snapped to.
                }

                remaining -= toi.t * remaining;
        }

        float vabs = vec3_dot(o->p.v, o->p.v);
        // is the object at rest?
        if (vabs < w->ss_thr && o->contact_mesh)
        {
                o->steady_state = 1;
        }
}

void vverlet_step(const struct world* w, struct object* o, float dt)
{
        struct vec3 f;

        if (o->contact_mesh)
        {
                float v_normal = vec3_dot(o->p.v, o->contact_normal);
                if (v_normal < 0.0f)
                {
                        // make sure object does not move into the surface
                        struct vec3 v_corr = vec3_scalarm(o->contact_normal,
                                                          v_normal);
                        o->p.v = vec3_sub(o->p.v, v_corr);
                }
        }

        // velocity half step (Verlet integration)
        o->p.v.x += o->p.a.x * dt * 0.5f;
        o->p.v.y += o->p.a.y * dt * 0.5f;
        o->p.v.z += o->p.a.z * dt * 0.5f;

        o->p.p.x += o->p.v.x * dt;
        o->p.p.y += o->p.v.y * dt;
        o->p.p.z += o->p.v.z * dt;

        // Resolve forces
        f.x = o->m * w->g.x;
        f.y = o->m * w->g.y;
        f.z = o->m * w->g.z;

        f = vec3_sub(f, drag_force(w, o));

        // apply normal force and friction
        if (o->contact_mesh)
        {
                f = vec3_add(f, friction_force_dyn(o->contact_mesh, o));

                float f_normal = vec3_dot(f, o->contact_normal);
                // The surface pushes back against forces pushing into it
                if (f_normal < 0.0f) {
                        struct vec3 f_corr = vec3_scalarm(o->contact_normal,
                                                          f_normal);
                        f = vec3_sub(f, f_corr);
                }
        }

        // compute new accelerations
        o->p.a.x = f.x / o->m;
        o->p.a.y = f.y / o->m;
        o->p.a.z = f.z / o->m;

        // velocity take two
        o->p.v.x += o->p.a.x * dt * 0.5f;
        o->p.v.y += o->p.a.y * dt * 0.5f;
        o->p.v.z += o->p.a.z * dt * 0.5f;
}

void collide_object(struct mesh* m,
                    struct object* o,
                    struct vec3 n,
                    float vn)
{
        if (vn > 0.0)
        {
                // object is moving away from the surface
                // should never happen
                fprintf(stderr, "WARNING: collision when moving away from a surface vn: %f \n", vn);
                fprintf(stderr, "p.p %f %f %f\n", o->p.p.x, o->p.p.y, o->p.p.z);
                fprintf(stderr, "p.v %f %f %f\n", o->p.v.x, o->p.v.y, o->p.v.z);
                fprintf(stderr, "n %f %f %f\n", n.x, n.y, n.z);

                return;
        }

        struct vec3 vt = vec3_sub(o->p.v, vec3_scalarm(n, vn));
        float rc = sqrtf(m->restitution * o->restitution);
        float mu = sqrtf(m->dynamic_mu * o->dynamic_mu);

        // Apply Coulomb friction to the tangential velocity.
        float factor = (1.0f + rc) * vn;

        if (!vec3_iszero(vt))
        {
                float jn = -factor * o->m;
                struct vec3 vt_dir = vec3_norm(vt);
                float jf = MIN(mu * fabsf(jn), CCR * o->m * sqrtf(vec3_dot(vt, vt)));

                struct vec3 res = vec3_scalarm(vt_dir, jf * o->m_inv);
                o->p.v = vec3_sub(o->p.v, res);
        }

        // as n is normalized, vn is the velocity of the particle.
        // Compute the scaling factor with the restitution, and
        // reduce speed in the scaled normal's direction.
        // Note that vn is reused from the coulomb friction calculation,
        // this is fine as it only updates the tangential velocity.
        struct vec3 ns = vec3_scalarm(n, factor);
        o->p.v = vec3_sub(o->p.v, ns);
}

struct vec3 drag_force(const struct world* w, const struct object* o)
{
        struct vec3 f;
        float half_rho = w->air_density * 0.5f;
        float cd = o->area * o->drag_c;
        float vs = sqrtf(vec3_dot(o->p.v, o->p.v));

        f.x = half_rho * vs * o->p.v.x * cd;
        f.y = half_rho * vs * o->p.v.y * cd;
        f.z = half_rho * vs * o->p.v.z * cd;

        return f;
}

struct vec3 friction_force_dyn(const struct mesh* m, const struct object* o)
{
        struct vec3 up = (struct vec3){ .a = {0.0f, 1.0f, 0.0f } };
        struct vec3 tv;
        float mu;
        float fN = o->m * KM_PHYS_G * vec3_dot(up, o->contact_normal);
        float vn;

        fN = fabsf(fN);
        mu = sqrtf(m->dynamic_mu * o->dynamic_mu);

        // compute the tangent velocity
        vn = vec3_dot(o->contact_normal, o->p.v);
        tv = vec3_scalarm(o->contact_normal, vn);
        tv = vec3_sub(o->p.v, tv);
        // only normalize if the tangent velocity
        // is not zero
        if (!vec3_iszero(tv))
        {
                tv = vec3_norm(tv);
        }

        // Friction force always oposes motion
        return vec3_scalarm(tv, -fN * mu);
}

struct vec3 friction_force_coulomb(const struct mesh* m,
                                   const struct object* o,
                                   struct vec3 n)
{
        float vn = vec3_dot(n, o->p.v);
        struct vec3 vt = vec3_sub(o->p.v, vec3_scalarm(n, vn));
        if (vec3_iszero(vt))
        {
                struct vec3 f = {0};
                return f;
        }

        float rc = sqrtf(m->restitution * o->restitution);
        float mu = sqrtf(m->dynamic_mu * o->dynamic_mu);
        float jn = -(1.0f + rc) * vn * o->m;
        struct vec3 vt_dir = vec3_norm(vt);
        float jf = MIN(mu * fabsf(jn), CCR * o->m * sqrtf(vec3_dot(vt, vt)));

        // only update the friction part, collision resolving updates
        // the velocity to move away from the surface
        struct vec3 res = vec3_scalarm(vt_dir, jf * o->m_inv);

        return res;
}

float friction_force_stat(const struct mesh* m, const struct object* o)
{
        struct vec3 up = (struct vec3){ .a = {0.0f, 1.0f, 0.0f } };
        float mu = sqrtf(m->static_mu * o->static_mu);
        float fN = o->m * KM_PHYS_G * vec3_dot(up, o->contact_normal);

        return fabsf(fN) * mu;
}

void quantize_force(struct vec3* f)
{
        float thr = 0.01f;

        if (fabsf(f->x) < thr) f->x = 0.0f;
        if (fabsf(f->y) < thr) f->y = 0.0f;
        if (fabsf(f->z) < thr) f->z = 0.0f;
}

int init_water(struct water* w, struct mesh* v, struct mesh* d)
{
        float d1 = v->vertices[1].pos.x - v->vertices[0].pos.x;

        if (v->grid_x == 0 || v->grid_z == 0)
        {
                return -1;
        }
        if (d)
        {
                if (v->grid_x != d->grid_x ||
                    v->grid_z != d->grid_z)
                {
                        return -1;
                }
        }

        w->h = d1;
        w->d = d;

        w->c = 1.5f; // wave propagation of 1.5m/s
        w->z = malloc(v->vertex_count * sizeof(struct vertex));

        // copy the vertices as is
        memcpy(w->z, v->vertices, v->vertex_count * sizeof(struct vertex));

        return 0;
}

void update_water(struct water* w, struct mesh* v, float dt)
{
        struct vertex* tmp;
        int stride = v->grid_x;
        float a = (w->c * dt) / w->h;
        float b;
        float s;

        a = a * a;
        b = 2.0f - 4.0f * a;

        s = w->c * w->c * dt * dt;
        s = s / (w->h * w->h);

        if (s > 0.5f)
        {
                printf("instability detected using explicit integration\n");
                printf("a: %f b: %f c: %f dt: %f h: %f\n",
                       a, b, w->c, dt, w->h);
                return;
        }

        // swap vertex pointers
        tmp = w->z;
        w->z = v->vertices;
        v->vertices = tmp;

        for (int y = 1; y < v->grid_z - 1; y++)
        {
                for (int x = 1; x < v->grid_x - 1; x++)
                {
                        float new =
                                a * (w->z[(y-1) * stride + x].pos.y +
                                     w->z[(y+1) * stride + x].pos.y +
                                     w->z[y * stride + x - 1].pos.y +
                                     w->z[y * stride + x + 1].pos.y);
                        float new2 = b * w->z[y * stride + x].pos.y -
                                v->vertices[y * stride + x].pos.y;
                        new += new2;

                        float gh = w->d->vertices[y * stride + x].pos.y;

                        float d;

                        if (gh > 0.299f)
                        {
                                d = 0.0f;
                        }
                        else if (gh < -0.2999f)
                        {
                                d = 1.0f;
                        }
                        else
                        {
                                d = -0.6f * gh + 0.5f;
                        }

                        v->vertices[y * stride + x].pos.y = d * new;
                }
        }
}
