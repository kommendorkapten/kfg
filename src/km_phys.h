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

#ifndef KM_PHYS_H
#define KM_PHYS_H

#include "km_math.h"

struct object;
struct mesh;

// m/s2
#define KM_PHYS_G 9.818f
// kg/m3
#define KM_PHYS_AIR_DENS 1.225f

struct world
{
        // Gravitation constant
        struct vec3 g;
        // Time duration of each integration step
        float dt;
        // Air density to use
        float air_density;
        // Any surfaces in the world
        struct mesh* surfaces;
        // Number of meshes
        int surface_count;
        // threshod for squared velocity to considered to be in a steady state
        float ss_thr;
        // treshold for squared velocity during collitions
        float ss_c_thr;
};

/**
 * Create a default (empty) world
 * @param the world struct to initialize
 * @param frame rate
 * @return void
 */
extern void default_world(struct world*, int);

/**
 * Run one update step for all objects using the provided world.
 * @param the current step
 * @param the world instance to use
 * @param the objects to update
 * @param number of objects
 * @param print the object (0/1)
 * @return void
 */
extern void update_objects(int, struct world*, struct object*, int, char);

/**
 * Run one update step for one objects using the provided world.
 * @param the current step
 * @param the world instance to use
 * @param the object to update
 * @return void
 */
extern void update_object(int step, struct world* w, struct object* o);

/**
 * Quantize forces near zero to zero.
 * @param The force vector to quantize
 * @return void
 */
extern void quantize_force(struct vec3*);

/**
 * Apply the drag force to an object
 * @param the object's force vector to update with the computed drag force
 * @param the the world instance to use
 * @param the object to compute draf force for
 * @return void
 */
extern void drag_force(struct vec3*,
                       const struct world*,
                       const struct object*);

#endif /* KM_PHYS_H */
