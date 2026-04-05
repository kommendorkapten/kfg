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

#define MAX_CONTACT_DIST 0.002f

struct particle;

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
        // three per triangle, indexed in the same way as indices
        struct vec3* inward_normals;
        float restitution;
        // static friction coefficient
        float static_mu;
        // dynamic friction coefficient
        float dynamic_mu;
        uint16_t vertex_count;
        uint32_t index_count;
        // Indices to the triangles, in CCW
        // the vertices are i * 3 + 0,1,2
        uint16_t* indices;
        // If the mesh is rectangle, these are the number of vertices
        // in each direction.
        uint16_t grid_x;
        uint16_t grid_z;
};

struct collision
{
        struct vec3 n;
        float t;
        struct mesh* m;
        uint32_t ti;
};

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
                  uint32_t);

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
 * Find the mesh that the particle p first will collide with.
 * @param t the toi to populate
 * @param p the particle
 * @param m an array of meshes to test against
 * @param mc mesh count
 * @return 1 if a collision happend, 0 otherwise
 */
int compute_toi(struct collision* t,
                struct particle* p,
                struct mesh** m,
                unsigned int mc);

/**
 * Test if a position is on or just above the mesh.
 * This iterate through all triangles and performs a check
 * for each triangle if the point is on or just above the
 * surface.
 * @param m the mesh to test against
 * @param p the point
 * @return 1 is on or just above the mesh, otherwise 0
 */
int point_on_mesh(struct mesh* m, struct vec3 p);

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

/**
 * Generate a rectangular mesh with equidistant vertices.
 * The mesh is rooted at (0, 0, 0) and constructed along the x and z axis
 * v0  v1 v2
 *  *--*--*
 *  | /| /|
 *  |/ |/ |
 *  *--*--*
 * v3  v4 v5
 * etc
 * @param x the width of the mesh
 * @param z the depth of the mesh
 * @param d axis aligned distance between each vertex
 * @return a mesh or NULL
 */
struct mesh* gen_mesh(float x, float z, float d);

/**
 * Recreate the normal for each vertex by iterating over all triangles
 * and adding the surface normal to each vertex.
 * @param m the mesh to create normals for
 * @return void
 */
void mesh_normalize(struct mesh* m);

/**
 * Translate the mesh by the provided vector
 * @param m the mesh to translate
 * @param v the offset
 * @return void
 */
void mesh_translate(struct mesh* m, struct vec3 v);

/**
 * Generate inward pointing normals for each edge for each triangle.
 * @param m the mesh to update with inward pointing normals
 * @return void
 */
void mesh_inward_normalize(struct mesh* m);

/**
 * Generate a heightmap on a mesh using scattered peaks with falloff.
 * Normals are recreated once the height map is done.
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
