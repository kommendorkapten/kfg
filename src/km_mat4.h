/*
 * 4×4 matrix utilities (column-major, matching Metal / OpenGL layout).
 *
 * Column-major flat-array layout:
 *
 *     m[0]  m[4]  m[8]   m[12]
 *     m[1]  m[5]  m[9]   m[13]
 *     m[2]  m[6]  m[10]  m[14]
 *     m[3]  m[7]  m[11]  m[15]
 *
 * All angles are in radians.
 */

#ifndef KFG_KM_MAT4_H
#define KFG_KM_MAT4_H

#include "km_math.h"

/**
 * Create an identity matrix
 * @param m the matrix to initialize
 * @return void
 */
extern void mat4_identity(float* m);

/**
 * Multiply two matricies. Safe to use when the result aliase a or b
 * r = a * b
 * @param r the result
 * @param a
 * @param b
 */
void mat4_multiply(float* r, const float* a, const float* b);

/**
 * Create a perspective projection for Metal's clip-space depth range [0, 1]
 * @param m created projection matrix
 * @param fovy vertical field-of-view in radians.
 * @param aspect ratio (width / height)
 * @param near near clipping plane, typically 0.1f
 * @param far far clipping plane, typicall 100.0f
 * @return void
 */
void mat4_perspective(float* m,
                      float fovy,
                      float aspect,
                      float near,
                      float far);

/**
 * Create a view matrix
 * @param m the created view matrix
 * @param eye the camera position in world space
 * @param center the look-at target in world space
 * @param up the a vector representing up in world space (typically 0,1,0)
 * @return void
 */
void mat4_look_at(float* m,
                  const struct vec3* eye,
                  const struct vec3* center,
                  const struct vec3* up);

/**
 * Create a rotation around the x axis
 * @param m the output rotation matrix
 * @param angle the angle to rotate (radians)
 * @return void
 */
void mat4_rotate_x(float* m, float angle);

/**
 * Create a rotation around the y axis
 * @param m the output rotation matrix
 * @param angle the angle to rotate (radians)
 * @return void
 */
void mat4_rotate_y(float* m, float angle);

/**
 * Create a rotation around the z axis
 * @param m the output rotation matrix
 * @param angle the angle to rotate (radians)
 * @return void
 */
void mat4_rotate_z(float* m, float angle);

void mat4_print(const float* m);


#endif /* KFG_KM_MAT4_H */
