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

#include <stdio.h>
#include <stdint.h>
#include <math.h>
#include "km_math.h"

_Static_assert(sizeof(struct vec3) == 12, "vec3 must be 12 bytes");
_Static_assert(sizeof(struct vec4) == 16, "vec3 must be 16 bytes");

struct vec3 vec3_add(struct vec3 v0,
                     struct vec3 v1)
{
        struct vec3 r;

        r.x = v0.x + v1.x;
        r.y = v0.y + v1.y;
        r.z = v0.z + v1.z;

        return r;
}

struct vec3 vec3_sub(struct vec3 v0,
                     struct vec3 v1)
{
        struct vec3 r;

        r.x = v0.x - v1.x;
        r.y = v0.y - v1.y;
        r.z = v0.z - v1.z;

        return r;
}

struct vec3 vec3_scalarm(struct vec3 v, float s)
{
        struct vec3 r;

        r.x = v.x * s;
        r.y = v.y * s;
        r.z = v.z * s;

        return r;
}

struct vec3 vec3_norm(struct vec3 v)
{
        // try with rsqrt intrinsic + one NR, should give same precision
        // float x = vec3_dot(v, v)
        // float rsq = __builtin_ia32_rsqrtss(dot);
        // return rsq * (1.5f - 0.5f * x * rsq * rsq)
        float rsq = 1.0f / sqrtf(vec3_dot(v, v));
        return vec3_scalarm(v, rsq);
}


float vec3_dot(struct vec3 v0,
               struct vec3 v1)
{
        return v0.x * v1.x + v0.y * v1.y + v0.z * v1.z;
}

struct vec3 vec3_cross(struct vec3 v0,
                       struct vec3 v1)
{
        struct vec3  r;

        r.x = v0.y * v1.z - v0.z * v1.y;
        r.y = v0.z * v1.x - v0.x * v1.z;
        r.z = v0.x * v1.y - v0.y * v1.x;

        return r;
}

float km_rsqrt(float x)
{
        union {
                float f;
                uint32_t i;
        } conv = {.f = x};

        conv.i = 0x5f3759df - (conv.i >> 1);  // Initial estimate

        // Newton-Raphson iteration
        conv.f *= 1.5f - (0.5f * x * conv.f * conv.f);

        return conv.f;
}

void vec3_print(struct vec3 v)
{
        printf("x:%f y:%f z:%f\n", v.x, v.y, v.z);
}

int vec3_iszero(struct vec3 v)
{
        if (vec3_dot(v, v) < 1e-8f)
        {
                return 1;
        }

        return 0;
}
