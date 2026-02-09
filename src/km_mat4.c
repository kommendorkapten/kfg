#include <stdio.h>
#include <math.h>
#include <string.h>
#include "km_mat4.h"

void mat4_identity(float* m)
{
        memset(m, 0, 16 * sizeof(float));
        m[0] = 1.0f; m[5] = 1.0f; m[10] = 1.0f; m[15] = 1.0f;
}

void mat4_multiply(float* r, const float* a, const float* b)
{
        float tmp[16];
        int i, j, k;

        for (j = 0; j < 4; j++)
        {
                for (i = 0; i < 4; i++)
                {
                        float sum = 0.0f;
                        for (k = 0; k < 4; k++)
                                sum += a[k * 4 + i] * b[j * 4 + k];
                        tmp[j * 4 + i] = sum;
                }
        }
        memcpy(r, tmp, sizeof(tmp));
}

void mat4_perspective(float* m,
                      float fovy,
                      float aspect,
                      float near,
                      float far)
{
        float f = 1.0f / tanf(fovy * 0.5f);

        memset(m, 0, 16 * sizeof(float));
        m[0]  = f / aspect;
        m[5]  = f;
        m[10] = far / (near - far);
        m[11] = -1.0f;
        m[14] = (near * far) / (near - far);
}

void mat4_look_at(float* m,
                  const struct vec3* eye,
                  const struct vec3* center,
                  const struct vec3* up)
{
        /* forward = normalize(center - eye) */
        float fx = center->x - eye->x;
        float fy = center->y - eye->y;
        float fz = center->z - eye->z;
        float flen = sqrtf(fx * fx + fy * fy + fz * fz);
        fx /= flen;  fy /= flen;  fz /= flen;

        /* side = normalize(cross(forward, up)) */
        float sx = fy * up->z - fz * up->y;
        float sy = fz * up->x - fx * up->z;
        float sz = fx * up->y - fy * up->x;
        float slen = sqrtf(sx * sx + sy * sy + sz * sz);
        sx /= slen;  sy /= slen;  sz /= slen;

        /* u = cross(side, forward) */
        float ux = sy * fz - sz * fy;
        float uy = sz * fx - sx * fz;
        float uz = sx * fy - sy * fx;

        m[0]  =  sx;  m[4]  =  sy;  m[8]  =  sz;
        m[12] = -(sx * eye->x + sy * eye->y + sz * eye->z);

        m[1]  =  ux;  m[5]  =  uy;  m[9]  =  uz;
        m[13] = -(ux * eye->x + uy * eye->y + uz * eye->z);

        m[2]  = -fx;  m[6]  = -fy;  m[10] = -fz;
        m[14] =  (fx * eye->x+ fy * eye->y + fz * eye->z);

        m[3]  = 0.0f; m[7]  = 0.0f; m[11] = 0.0f; m[15] = 1.0f;
}

void mat4_rotate_x(float* m, float angle)
{
        float c = cosf(angle);
        float s = sinf(angle);

        memset(m, 0, 16 * sizeof(float));
        m[0]  =  1.0f;
        m[5]  =  c;    m[9]  = -s;
        m[6]  =  s;    m[10] =  c;
        m[15] =  1.0f;
}

void mat4_rotate_y(float* m, float angle)
{
        float c = cosf(angle);
        float s = sinf(angle);

        memset(m, 0, 16 * sizeof(float));
        m[0]  =  c;    m[8]  =  s;
        m[5]  =  1.0f;
        m[2]  = -s;    m[10] =  c;
        m[15] =  1.0f;
}

void mat4_rotate_z(float* m, float angle)
{
        float c = cosf(angle);
        float s = sinf(angle);

        memset(m, 0, 16 * sizeof(float));
        m[0]  =  c;    m[4]  = -s;
        m[1]  =  s;    m[5]  =  c;
        m[10] =  1.0f;
        m[15] =  1.0f;
}

void mat4_translate(float *m, float x, float y, float z)
{
        mat4_identity(m);
        m[12] = x;
        m[13] = y;
        m[14] = z;
}

void mat4_print(const float* m)
{
        printf("(%f %f %f %f\n", m[0], m[4], m[8], m[12]);
        printf("(%f %f %f %f\n", m[1], m[5], m[9], m[13]);
        printf("(%f %f %f %f\n", m[2], m[6], m[10], m[14]);
        printf("(%f %f %f %f\n", m[3], m[7], m[11], m[15]);
}
