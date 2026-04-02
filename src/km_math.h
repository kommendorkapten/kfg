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

#ifndef MIN
# define MIN(a, b) ((a) < (b) ? (a) : (b))
#endif
#ifndef MAX
# define MAX(a, b) ((a) > (b) ? (a) : (b))
#endif
#ifndef M_PI
#define M_PI 3.141592653589793238462643383279502884
#endif

struct vec3
{
        union
        {
                float a[3];
                struct
                {
                        float x;
                        float y;
                        float z;
                };
        };
};

struct vec4
{
        union
        {
                float a[4];
                struct
                {
                        float x;
                        float y;
                        float z;
                        float w;
                };
        };
};

struct vec3 vec3_add(struct vec3, struct vec3);
struct vec3 vec3_sub(struct vec3, struct vec3);
struct vec3 vec3_scalarm(struct vec3, float);
/**
 * Normalize a vector
 * @param v the vector be to normalize
 * @return the noralized vector
 */
struct vec3 vec3_norm(struct vec3);
float vec3_dot(struct vec3, struct vec3);
struct vec3 vec3_cross(struct vec3, struct vec3);
float km_rsqrt(float x);
void vec3_print(struct vec3 v);
int vec3_iszero(struct vec3 v);

#endif /* KM_MATH_H */
