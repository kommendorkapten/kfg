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

#ifndef KM_MATH_H
#define KM_MATH_H

struct vec3
{
        float x;
        float y;
        float z;
};

extern struct vec3 vec3_add(const struct vec3*, const struct vec3*);
extern struct vec3 vec3_sub(const struct vec3*, const struct vec3*);
extern struct vec3 vec3_scalarm(const struct vec3*, float);
/**
 * Normalize a vector
 * @param v the vector be to normalize
 * @return the noralized vector
 */
extern struct vec3 vec3_norm(const struct vec3*);
extern float vec3_dot(const struct vec3*, const struct vec3*);
extern struct vec3 vec3_cross(const struct vec3* restrict,
                              const struct vec3* restrict);
extern float km_rsqrt(float x);

#endif /* KM_MATH_H */
