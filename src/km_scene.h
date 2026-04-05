#ifndef KM_SCENE_H
#define KM_SCENE_H

#include "km_phys.h"
#include "km_geom.h"

struct animation {
        uint32_t current_frame;
        uint32_t state;
        float time;
        float speed;
};

struct entity;

typedef void (*animate_fn)(struct entity* e, float dt);

struct entity
{
        struct object o;
        struct animation a;
        animate_fn animate;
        struct mesh* surfaces;
        unsigned int surface_count;
};

struct camera
{
        // Position of the camera
        struct vec3 pos;
        // Point camera is pointing at
        struct vec3 center;
        // Up vector
        struct vec3 up;
};

struct scene
{
        struct world w;
        struct camera cam;
        struct entity** entities;
        unsigned int entity_count;
        unsigned int entity_cap;
};

void scene_init(struct scene* s);

void scene_free(struct scene* s);

/**
 * Add an entity and increment the entity count
 * @param w the world
 * @param m the mesh to add as a surface
 * @return void
 */
void scene_add_entity(struct scene* s, struct entity* e);

/**
 * Animate rotation around the X axis.
 * Uses the entity's animation speed as angular velocity (rad/s).
 */
void animate_rot_x(struct entity* e, float dt);

/**
 * Animate rotation around the Y axis.
 * Uses the entity's animation speed as angular velocity (rad/s).
 */
void animate_rot_y(struct entity* e, float dt);

/**
 * Animate rotation around the Z axis.
 * Uses the entity's animation speed as angular velocity (rad/s).
 */
void animate_rot_z(struct entity* e, float dt);

#endif /* KM_SCENE_H */
