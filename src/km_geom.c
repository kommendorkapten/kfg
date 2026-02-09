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

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "km_geom.h"

void print_particle(const struct particle* p)
{
        printf("pos: %f %f %f\n", p->p.x, p->p.y, p->p.z);
        printf("vel: %f %f %f\n", p->v.x, p->v.y, p->v.z);
        printf("acl: %f %f %f\n", p->a.x, p->a.y, p->a.z);
}

void mesh_get_tri(struct vertex** restrict v0,
                  struct vertex** restrict v1,
                  struct vertex** restrict v2,
                  const struct mesh* m,
                  int i)
{
        *v0 = m->vertices + m->indices[i * 3 + 0];
        *v1 = m->vertices + m->indices[i * 3 + 1];
        *v2 = m->vertices + m->indices[i * 3 + 2];
}

int ray_tri_intersect(const struct particle* r,
                      const struct vec3* v0,
                      const struct vec3* v1,
                      const struct vec3* v2,
                      float* t,
                      float* u,
                      float* v)
{
        const float epsilon = 1e-6f;
        struct vec3 e1;
        struct vec3 e2;
        struct vec3 p;
        struct vec3 s;
        struct vec3 q;
        float f;
        float inv_f;

        e1 = vec3_sub(v1, v0);
        e2 = vec3_sub(v2, v0);
        p = vec3_cross(&r->v, &e2);

        f = vec3_dot(&e1, &p);
        if (fabs(f) < epsilon)
        {
                // Ray is parallel to the triangle's plane
                return 0;
        }

        inv_f = 1.0f / f;
        s = vec3_sub(&r->p, v0);
        *u = inv_f * vec3_dot(&s, &p);
        if (*u < 0.0f || *u > 1.0f)
        {
                return 0;
        }

        q = vec3_cross(&s, &e1);
        *v = inv_f * vec3_dot(&r->v, &q);
        if (*v < 0.0f || *u + *v > 1.0f)
        {
                return 0;
        }

        *t = inv_f * vec3_dot(&e2, &q);
        if (*t < epsilon)
        {
                // particle is already behind triangle
                return 0;
        }

        return 1;
}

void collide_particle(struct particle* p,
                      struct vec3* restrict v0,
                      struct vec3* restrict v1,
                      struct vec3* restrict v2,
                      float rc)
{
        struct vec3 e1 = vec3_sub(v1, v0);
        struct vec3 e2 = vec3_sub(v2, v0);
        struct vec3 n = vec3_cross(&e1, &e2);

        n = vec3_norm(&n);
        float vn = vec3_dot(&p->v, &n);

        if (vn > 0.0)
        {
                printf("p.p %f %f %f\n", p->p.x, p->p.y, p->p.z);
                printf("p.v %f %f %f\n", p->v.x, p->v.y, p->v.z);
                printf("n %f %f %f\n", n.x, n.y, n.z);
                printf("v0 %f %f %f\n", v0->x, v0->y, v0->z);
                printf("v1 %f %f %f\n", v1->x, v1->y, v1->z);
                printf("v2 %f %f %f\n", v2->x, v2->y, v2->z);

                // object is moving away from the surface
                // should never happen
                printf("WARNING: collision when moving away from a surface\n");
                return;
        }

        // as n is normalized, vn is the velocity of the particle.
        // Compute the scaling factor with the restitution, and
        // reduce speed in the scaled normal's direction
        float factor = (1.0f + rc) * vn;
        n = vec3_scalarm(&n, factor);
        p->v = vec3_sub(&p->v, &n);
}

void mesh_free(struct mesh* m)
{
        free(m->vertices);
        free(m->indices);
        memset(m, 0, sizeof(*m));
}
