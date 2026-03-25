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
struct vertex;

// m/s2
#define KM_PHYS_G 9.818f
// kg/m3
#define KM_PHYS_AIR_DENS 1.225f

struct particle
{
        // Position
        struct vec3 p;
        // Velocity
        struct vec3 v;
        // Acceleration
        struct vec3 a;
        // Rotation around each axis, in radians
        struct vec3 r;
        // Bounding sphere radius (0 = point particle)
        float rad;
};

struct object
{
        struct particle p;
        // The mass of the object. Prefer to call object_set_m(m)
        // as this will automatically update the m_inv
        float m;
        // 1/m.
        float m_inv;
        // The cross sectional area in the velocity direction
        float area;
        // the drag coefficient
        float drag_c;
        // Set to 1 if this object is not moving
        char steady_state;
        // restitution constant for collisions
        float restitution;
        // static friction coefficient
        float static_mu;
        // dynamic friction coefficient
        float dynamic_mu;
        // Persistent contact cache
        struct mesh* contact_mesh;
        struct vec3 contact_normal;
};

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
        // Any water in the world
        struct mesh* waters;
        // Number of meshes
        int water_count;
        // threshod for squared velocity to considered to be in a steady state
        float ss_thr;
};

struct water
{
        float c;
        float h;
        // the surface for which the water is above/under
        // used to apply as a damping mask to the wave updates
        struct mesh* d;
        // the buffer with the last state
        // Todo, replace witha simpel MxN float array?
        struct vertex* z;
};

/**
 * Print a particle to stdout
 * @param p the particle to print
 * @return void
 */
void print_particle(const struct particle* p);

/**
 * Update the mass of an object. This will also update the m_inv
 * @param m the object's new mass
 * @return void
 */
void object_set_m(struct object* o, float m);

/**
 * v and d must have the same spacing, and each vertex must have the
 * same x and z position.
 */
int init_water(struct water* w, struct mesh* v, struct mesh* d);

void update_water(struct water* w, struct mesh* v, float dt);

/**
 * Create a default (empty) world
 * @param the world struct to initialize
 * @param frame rate
 * @return void
 */
void default_world(struct world*, int);

/**
 * Run one update step for all objects using the provided world.
 * @param the current step
 * @param the world instance to use
 * @param the objects to update
 * @param number of objects
 * @param print the object (0/1)
 * @return void
 */
void update_objects(int, const struct world*, struct object*, int, char);

/**
 * Run one update step for one objects using the provided world.
 * @param the current step
 * @param the world instance to use
 * @param the object to update
 * @return void
 */
void update_object(int step, const struct world* w, struct object* o);

/**
 * Run one velocity verlet step for one objects using the provided world.
 * @param w the world instance to use
 * @param o the object to update
 * @param dt the time step
 * @return void
 */
void vverlet_step(const struct world* w, struct object* o, float dt);

/**
 * Collide an object with a surface
 * The objects's velocity is updated to bounce back in the direction
 * of the normal of the triangle, scaled with the provided restitution.
 * During the collision, Coulomb friction is computed and used to update
 * the tangential velocity.
 * @param m the mesh to collide with.
 * @param o the object to collide.
 * @param n the collision normal.
 * @param vn the velocity of the particle across the collision normal.
 * @return void
 */
void collide_object(struct mesh* m,
                    struct object* o,
                    struct vec3 n,
                    float vn);

/**
 * Quantize forces near zero to zero.
 * @param The force vector to quantize
 * @return void
 */
void quantize_force(struct vec3*);

/**
 * Compute the drag force to an object
 * @param the the world instance to use
 * @param the object to compute draf force for
 * @return the drag force
 */
struct vec3 drag_force(const struct world* w,
                       const struct object* o);

/**
 * Compute the dynamic friction force between an object and a surface.
 * @param the surface
 * @param the object
 * @return the friction force
*/
struct vec3 friction_force_dyn(const struct mesh* m,
                               const struct object* o);

/**
 * Compute the Coulomb friction impulse between an object and a surface,
 * to be used during a collision. The returned impulse vector should be
 * subtracted from the object's velocity.
 * @param m the surface
 * @param o the object
 * @param n the contact normal
 * @return the friction impulse to subtract from velocity.
*/
struct vec3 friction_force_coulomb(const struct mesh* m,
                                   const struct object* o,
                                   struct vec3 n);

/**
 * Compute the static friction force between an object and a surface.
 * @param the surface
 * @param the object
 * @return the friction force
*/
float friction_force_stat(const struct mesh* e,
                          const struct object* o);

#endif /* KM_PHYS_H */
