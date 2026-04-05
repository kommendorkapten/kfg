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
#include <string.h>
#include "km_scene.h"

void scene_init(struct scene* s)
{
        memset(s, 0, sizeof(*s));
        s->entity_cap = 128;
        s->entity_count = 0;
        s->entities = malloc(s->entity_cap * sizeof(struct entity*));
}

void scene_free(struct scene* s)
{
        free(s->entities);
}

void scene_add_entity(struct scene* s, struct entity* e)
{
        if (s->entity_count == s->entity_cap)
        {
                s->entity_cap <<= 1;
                struct entity** new = malloc(s->entity_cap *
                                             sizeof(struct entity*));
                memcpy(new, s->entities,
                       s->entity_count * sizeof(struct entity*));
                free(s->entities);
                s->entities = new;
        }
        s->entities[s->entity_count++] = e;
}

void animate_rot_x(struct entity* e, float dt)
{
        e->o.p.r.x += e->a.speed * dt;
}

void animate_rot_y(struct entity* e, float dt)
{
        e->o.p.r.y += e->a.speed * dt;
}

void animate_rot_z(struct entity* e, float dt)
{
        e->o.p.r.z += e->a.speed * dt;
}
