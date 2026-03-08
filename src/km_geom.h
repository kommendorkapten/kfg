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

#include <stdint.h>
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
        // Bounding sphere radius (0 = point particle)
        float rad;
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
        // static friction coefficient
        float static_mu;
        // dynamic friction coefficient
        float dynamic_mu;
        // Persistent contact cache
        struct mesh* contact_mesh;
        struct vec3 contact_normal;
};

struct vertex
{
        struct vec3 pos;
        struct vec3 normal;
        struct vec4 color;
};

/*
  Access triangle i. All triangles should be encoded in CCW order.
  *v0 = &mesh.vertices[mesh.indices[i * 3 + 0]];
  *v1 = &mesh.vertices[mesh.indices[i * 3 + 1]];
  *v2 = &mesh.vertices[mesh.indices[i * 3 + 2]];
*/
struct mesh
{
        struct vertex* vertices;
        uint16_t* indices;
        float restitution;
        // static friction coefficient
        float static_mu;
        // dynamic friction coefficient
        float dynamic_mu;
        uint16_t vertex_count;
        uint16_t index_count;
        // If the mesh is rectangle, these are the number of vertices
        // in each direction.
        uint16_t grid_x;
        uint16_t grid_z;
};

/**
 * Print a particle to stdout
 * @param p the particle to print
 * @return void
 */
void print_particle(const struct particle* p);

/**
 * Print a vertex to stdout
 * @param v the vertex to print
 * @return void
 */
void print_vertex(const struct vertex* v);

/**
 * Load a triangle (CCW) from the mesh
 * @params v0 the first vertex of the triangle
 * @params v0 the second vertex of the triangle
 * @params v0 the third vertex of the triangle
 * @params m the mesh to extract the triangle from
 * @params n the index of the triangle in the mesh
 * @return void
 */
void mesh_get_tri(struct vertex** restrict,
                  struct vertex** restrict,
                  struct vertex** restrict,
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
 * @param n the collision normal
 * @param vn the velocity of the particle across the collision normal
 * @param rc the combined restitution coefficent (prefer geometric mean)
 * @return void
 */
void collide_particle(struct particle* p,
                      struct vec3* n,
                      float vn,
                      float rc);

/**
 * Read the provided json file, and return an array of meshes.
 * @param p the path to the JSON file to read.
 * @param count the number of meshes read and returned
 * @return pointer to the meshes, or NULL if read failed.
 */
struct mesh* load_meshes(const char* p, int* count);

/**
 * Write an array of meshes to a JSON file.
 * @param p the path to the output JSON file.
 * @param meshes pointer to the array of meshes to write.
 * @param count the number of meshes in the array.
 * @return 0 on success, -1 on failure.
 */
int write_meshes(const char* p, const struct mesh* meshes, int count);

struct mesh* gen_mesh(float x, float y, float d);

void mesh_normalize(struct mesh* m);

void mesh_translate(struct mesh* m, struct vec3* v);

/**
 * Generate a heightmap on a mesh using scattered peaks with falloff.
 * @param m the mesh to apply heights to
 * @param peaks the number of random peaks to generate
 * @param max_height maximum height of any single peak
 * @param radius the influence radius of each peak
 * @return void
 */
void mesh_heightmap(struct mesh* m, int peaks, float max_height, float radius);

/**
 * Colorize a mesh based on vertex height (y).
 * Applies a smooth gradient from dark (low) through green,
 * brown, to sand (high).
 * @param m the mesh to colorize
 * @return void
 */
void mesh_colorize(struct mesh* m);

/**
 * Colorize a mesh with water colors based on vertex height (y).
 * Applies a smooth gradient from deep blue (low) through
 * ocean blue to light cyan (high).
 * @param m the mesh to colorize
 * @return void
 */
void mesh_colorize_water(struct mesh* m);

/**
 * Free all memory held by a mesh.
 * After the memory is freed, all members are set to zero.
 * If the mesh is allocated on the heap, it is not freed.
 * @param m a mesh to free
 * @return void
 */
void mesh_free(struct mesh* m);

#endif /* KM_GEOM_H */
