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

#ifndef KM_ENTITY_H
#define KM_ENTITY_H

#include "km_geom.h"

struct animation {
    uint32_t current_frame;
    uint32_t state;
    float time;
    float speed;
};

struct entity
{
        struct object o;
        struct mesh* surfaces;
        int num_surfaces;

};

#endif /* KM_ENTITY_H */
