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
        int surface_count;
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
        struct entity* entities;
        int entity_count;
};

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
