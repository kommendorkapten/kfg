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

#ifndef KM_GEOM_H
#define KM_GEOM_H

#include "km_math.h"

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
};

struct object
{
        struct particle p;
        // The mass of the object
        float m;
        // The cross sectional area in the velocity direction
        float area;
        // the drag coefficient
        float drag_c;
        // Set to 1 if this objevt is not moving
        char steady_state;
        // restitution constant for collisions
        float restitution;
};

/*
  Access triangle i. All triangles should be encoded in CCW order.
  *v0 = &mesh.vertices[mesh.indices[i * 3 + 0]];
  *v1 = &mesh.vertices[mesh.indices[i * 3 + 1]];
  *v2 = &mesh.vertices[mesh.indices[i * 3 + 2]];
*/
struct mesh
{
        struct vec3* vertices;
        int* indices;
        int vertex_count;
        int index_count;
        float restitution;
};

/**
 * Print a particle to stdout
 * @param p the particle to print
 * @return void
 */
void print_particle(const struct particle* p);

/**
 * Load a triangle (CCW) from the mesh
 * @params v0 the first vertex of the triangle
 * @params v0 the second vertex of the triangle
 * @params v0 the third vertex of the triangle
 * @params m the mesh to extract the triangle from
 * @params n the index of the triangle in the mesh
 * @return void
 */
void mesh_get_tri(struct vec3** restrict,
                  struct vec3** restrict,
                  struct vec3** restrict,
                  const struct mesh*,
                  int);

/**
 * Möller-Trumbore algorithm for particle (ray) triangle intersection
 * @param p the particle
 * @param v0, v1, v2 the vertices for the triangle (CCW)
 * @param t computed distance to intersection, in |p.v| units
 * @param u computed Barycentric u coord
 * @param v computed Barycentric v coord
 * @return 1 if the particle will intersect
 */
int ray_tri_intersect(const struct particle* r,
                      const struct vec3* v0,
                      const struct vec3* v1,
                      const struct vec3* v2,
                      float* t,
                      float* u,
                      float* v);

/**
 * Collide a particle with a triangle.
 * The particle's velocity is updated to bounce back in the direction
 * of the normal of the triangle, scaled with the provided restitution.
 * @param p the particle to collide
 * @param v0 the first vertex of the triangle (CCW)
 * @param v1 the second vertex of the triangle
 * @param v2 the third vertex of the triangle
 * @param rc the combined restitution coefficent (prefer geometric mean)
 * @return void
 */
void collide_particle(struct particle* p,
                      struct vec3* restrict v0,
                      struct vec3* restrict v1,
                      struct vec3* restrict v2,
                      float rc);

#endif /* KM_GEOM_H */
