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
#include "km_phys.h"
#include "km_math.h"
#include "km_geom.h"

void default_world(struct world* w, int fps)
{
        w->g = (struct vec3){ .a = {0.0f, -KM_PHYS_G, 0.0f} };
        w->dt = 1.0f / (float)fps;
        w->air_density = KM_PHYS_AIR_DENS;
        w->ss_thr   = 0.008f * 0.008f; // 8mm/s
        w->ss_c_thr = 0.08f * 0.08f; // 8cm/s
}

void update_objects(int step,
                    struct world* w,
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

void update_object(int step, struct world* w, struct object* o)
{
        struct particle p;
        struct vec3 new_pos;
        struct vec3 new_vel;
        struct vec3 f;
        float t_coll = 0;
        float remainder = 1.0f;
        int coll = 0;

        if (o->steady_state)
        {
                return;
        }

        (void)step;

        // velocity half step (Verlet integration)
        new_vel.x = o->p.v.x + o->p.a.x * w->dt * 0.5f;
        new_vel.y = o->p.v.y + o->p.a.y * w->dt * 0.5f;
        new_vel.z = o->p.v.z + o->p.a.z * w->dt * 0.5f;

        // position
        new_pos.x = o->p.p.x + new_vel.x * w->dt;
        new_pos.y = o->p.p.y + new_vel.y * w->dt;
        new_pos.z = o->p.p.z + new_vel.z * w->dt;

        // Copy old pos
        // Compute the collision against a particle with the
        // original position and velocity set to the step's
        // velocity, i.e new_pos - old_pos.
        p.p = o->p.p;
        p.v = vec3_sub(&new_pos, &p.p);

        // set external force to zero
        o->p.f = (struct vec3){ .a = { 0.0f, 0.0f, 0.0f } };

        // collision detection
        for (int s = 0; s < w->surface_count; s++)
        {
                struct mesh* m = w->surfaces + s;
                int num_tri = m->index_count / 3;

                for (int ti = 0; ti < num_tri; ti++)
                {
                        struct vertex* v0;
                        struct vertex* v1;
                        struct vertex* v2;
                        struct vec3 tmp;
                        float t;
                        float u;
                        float v;
                        int coll_test;

                        mesh_get_tri(&v0,
                                     &v1,
                                     &v2,
                                     m,
                                     ti);

                        // break out into new function
                        coll_test = ray_tri_intersect(&p,
                                                      &v0->pos,
                                                      &v1->pos,
                                                      &v2->pos,
                                                      &t,
                                                      &u,
                                                      &v);

                        /*
                          if a collision happens, it's measured in
                          units of the length, which is the current
                          step's computed velocity. The point of
                          intersection must be in this step.
                          For a sphere (radius > 0), the surface is
                          reached when the center is still radius
                          away, so we allow t up to
                          1 + radius / |displacement|.
                        */
                        if (coll_test)
                        {
                                float inv_disp_len = km_rsqrt(
                                        vec3_dot(&p.v, &p.v));
                                float t_max = remainder;
                                struct vec3 e1 = vec3_sub(&v1->pos, &v0->pos);
                                struct vec3 e2 = vec3_sub(&v2->pos, &v0->pos);
                                struct vec3 n = vec3_cross(&e1, &e2);
                                float v_normal;

                                n = vec3_norm(&n);

                                if (inv_disp_len < 1e5f &&
                                    o->p.rad > 0.0f) {
                                        // this should be
                                        // t = t - o->p.rad * inv_disp_ln
                                        // verify this
                                        t_max += o->p.rad * inv_disp_len;
                                }

                                if (t > t_max)
                                {
                                        // too far away
                                        continue;
                                }

                                t_coll = t * remainder;
                                coll = 1;

                                // Before updating the velocity move the
                                // object to the surface.
                                // The distance is t if radius is zero,
                                // t_max - t if radius is > 0
                                if (o->p.rad > 0.0f)
                                {
                                        t = t_max - t;
                                }

                                // The collision happens at time t * w->dt,
                                // not w->dt wihich is used to integrate the
                                // velocity, update the velocity to the
                                // expected velocity at impact.
                                o->p.v.x += o->p.a.x * w->dt * 0.5f * t_coll;
                                o->p.v.y += o->p.a.y * w->dt * 0.5f * t_coll;
                                o->p.v.z += o->p.a.z * w->dt * 0.5f * t_coll;

                                v_normal = vec3_dot(&o->p.v, &n);

                                // Collide the particle. This will mirror
                                // the velocity in the collision normal's
                                // direction and apply the restitution damping
                                float rc = sqrtf(m->restitution *
                                                 o->restitution);
                                collide_particle(&o->p,
                                                 &n,
                                                 v_normal,
                                                 rc);

                                // add the rest of the velocity step
                                o->p.v.x += o->p.a.x * w->dt * 0.5f * (remainder - t_coll);
                                o->p.v.y += o->p.a.y * w->dt * 0.5f * (remainder - t_coll);
                                o->p.v.z += o->p.a.z * w->dt * 0.5f * (remainder - t_coll);

                                // recompute v_normal as it's now different
                                // after the bounce
                                v_normal = vec3_dot(&o->p.v, &n);

                                // Move object to the collision surface
                                // todo: this shadows float v
                                struct vec3 v = vec3_scalarm(&p.v, t);
                                o->p.p = vec3_add(&o->p.p, &v);

                                // Move the object away from the surface
                                // a tiny bit (1mm)
                                struct vec3 ns = vec3_scalarm(&n, 0.001f);
                                o->p.p = vec3_add(&o->p.p, &ns);

                                // check if the bounce height is negligible
                                // the apex is when when 0.5 m v^2 = mgh
                                // (kinetic energy equals potential energy)
                                float bounce_h = (v_normal * v_normal) /
                                        (2.0f * KM_PHYS_G);
                                if (bounce_h < 0.002f)
                                {
                                        // less than 2mm, set the velocity in the normal's
                                        // direction to zero
                                        tmp = vec3_scalarm(&n, v_normal);
                                        o->p.v = vec3_sub(&o->p.v, &tmp);
                                        v_normal = 0.0f;
                                }
                                // TODO add quantize for the velocity

                                // apply dynamic friction
                                tmp = (struct vec3){ .a = {0.0f, 1.0f, 0.0f } };
                                float fN = o->m * KM_PHYS_G *
                                        fabsf(vec3_dot(&n, &tmp));
                                // compute the tangent velocity
                                tmp = vec3_scalarm(&n, v_normal);
                                tmp = vec3_sub(&o->p.v, &tmp);
                                // only normalize if the tangent velocity
                                // is not zero
                                if (!vec3_iszero(tmp))
                                {
                                        tmp = vec3_norm(&tmp);
                                }

                                float mu = sqrtf(o->dynamic_mu *
                                                 m->dynamic_mu);
                                tmp = vec3_scalarm(&tmp, -fN * mu);
                                o->p.f = vec3_add(&o->p.f, &tmp);

                                float vabs = vec3_dot(&o->p.v, &o->p.v);
                                // is the object at rest?
                                if (vabs < w->ss_c_thr)
                                {
                                        o->steady_state = 1;
                                        break;
                                }

                                remainder -= t_coll;
                                // we have collided with this mesh, goto next
                                break;
                        }
                }
        }
        if (coll)
        {
                // Note: this is bug. The current code assumes
                // that it will only collide with one surface per step
                // and so t_coll is the first collision time. If multiple
                // surfaces are hit during the same step this will increase
                // the energy of the particle too much.

                // Integrate the remainding time from after the collision
                float dt = (1.0f - t_coll) * w->dt;

                o->p.p.x = o->p.p.x + o->p.v.x * dt;
                o->p.p.y = o->p.p.y + o->p.v.y * dt;
                o->p.p.z = o->p.p.z + o->p.v.z * dt;
        }
        else
        {
                // Update position and velocity if there was no collision
                o->p.p = new_pos;
                o->p.v = new_vel;
        }

        // compute all forces
        // gravity
        f.x = o->m * w->g.x;
        f.y = o->m * w->g.y;
        f.z = o->m * w->g.z;

        // add computed forces from collision
        f = vec3_add(&f, &o->p.f);

        drag_force(&f, w, o);

        // compute new accelerations
        o->p.a.x = f.x / o->m;
        o->p.a.y = f.y / o->m;
        o->p.a.z = f.z / o->m;

        // velocity take two
        o->p.v.x += o->p.a.x * w->dt * 0.5f;
        o->p.v.y += o->p.a.y * w->dt * 0.5f;
        o->p.v.z += o->p.a.z * w->dt * 0.5f;
}

void drag_force(struct vec3* f, const struct world* w, const struct object* o)
{
        float half_rho = w->air_density * 0.5f;
        float cd = o->area * o->drag_c;
        float vs = sqrtf(vec3_dot(&o->p.v, &o->p.v));

        f->x -= half_rho * vs * o->p.v.x * cd;
        f->y -= half_rho * vs * o->p.v.y * cd;
        f->z -= half_rho * vs * o->p.v.z * cd;
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
