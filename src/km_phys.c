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

#include <math.h>
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

                if (o->steady_state)
                {
                        continue;
                }

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
        struct vec3 f;
        int coll = 0;

        (void)step;

        // velocity half step (Verlet integration)
        o->p.v.x += o->p.a.x * w->dt * 0.5f;
        o->p.v.y += o->p.a.y * w->dt * 0.5f;
        o->p.v.z += o->p.a.z * w->dt * 0.5f;

        // position
        new_pos.x = o->p.p.x + o->p.v.x * w->dt;
        new_pos.y = o->p.p.y + o->p.v.y * w->dt;
        new_pos.z = o->p.p.z + o->p.v.z * w->dt;

        // Copy old pos
        // Compute the collision against a particle with the
        // original position and velocity set to the step's
        // velocity, i.e new_pos - old_pos.
        p.p = o->p.p;
        p.v = vec3_sub(&new_pos, &p.p);

        // collision detection
        for (int s = 0; s < w->num_surfaces; s++)
        {
                struct mesh* m = w->surfaces + s;
                int num_tri = m->index_count / 3;

                for (int ti = 0; ti < num_tri; ti++)
                {
                        struct vec3* v0;
                        struct vec3* v1;
                        struct vec3* v2;
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
                                                      v0,
                                                      v1,
                                                      v2,
                                                      &t,
                                                      &u,
                                                      &v);
                        /*
                          if a collision happens, it's measured in
                          units of the length, which is the current
                          step's computed velocity. The point of
                          intersection must be in this step
                        */
                        if (coll_test && t <= 1.0f)
                        {
                                float rc = sqrtf(m->restitution *
                                                 o->restitution);

                                coll = 1;
                                collide_particle(&o->p,
                                                 v0,
                                                 v1,
                                                 v2,
                                                 rc);

                                // is the object at rest?
                                float vabs = vec3_dot(&o->p.v, &o->p.v);

                                if (vabs < w->ss_c_thr)
                                {
                                        o->steady_state = 1;
                                }
                                break;
                        }
                }
                if (coll)
                {
                        break;
                }
        }
        if (!coll)
        {
                // Update position if there was no collision
                o->p.p = new_pos;
        }

        // compute all forces
        // gravity
        f.x = o->m * w->g.x;
        f.y = o->m * w->g.y;
        f.z = o->m * w->g.z;

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

        f->x -= half_rho * fabsf(o->p.v.x) * o->p.v.x * cd;
        f->y -= half_rho * fabsf(o->p.v.y) * o->p.v.y * cd;
        f->z -= half_rho * fabsf(o->p.v.z) * o->p.v.z * cd;
}

void quantize_force(struct vec3* f)
{
        float thr = 0.01f;

        if (fabsf(f->x) < thr) f->x = 0.0f;
        if (fabsf(f->y) < thr) f->y = 0.0f;
        if (fabsf(f->z) < thr) f->z = 0.0f;
}
