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

#include <stdlib.h>
#include <math.h>
#include "common.h"
#include "km_geom.h"

void init_cube(struct mesh* m)
{
        m->vertex_count = 24;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 36;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->restitution = 1.0f;
        m->static_mu = 0.7f;
        m->dynamic_mu = 0.45f;

        /* Front face  (z = +1)  normal ( 0,  0,  1) */
        m->vertices[ 0] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 1] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 2] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 0,  0,  1} } };
        m->vertices[ 3] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = { 0,  0,  1} } };
        /* Back face   (z = -1)  normal ( 0,  0, -1) */
        m->vertices[ 4] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 5] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 6] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = { 0,  0, -1} } };
        m->vertices[ 7] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 0,  0, -1} } };
        /* Top face    (y = +1)  normal ( 0,  1,  0) */
        m->vertices[ 8] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[ 9] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[10] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[11] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = { 0,  1,  0} } };
        /* Bottom face (y = -1)  normal ( 0, -1,  0) */
        m->vertices[12] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[13] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[14] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 0, -1,  0} } };
        m->vertices[15] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = { 0, -1,  0} } };
        /* Right face  (x = +1)  normal ( 1,  0,  0) */
        m->vertices[16] = (struct vertex){ .pos = { .a = { 1, -1,  1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[17] = (struct vertex){ .pos = { .a = { 1, -1, -1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[18] = (struct vertex){ .pos = { .a = { 1,  1, -1} }, .normal = { .a = { 1,  0,  0} } };
        m->vertices[19] = (struct vertex){ .pos = { .a = { 1,  1,  1} }, .normal = { .a = { 1,  0,  0} } };
        /* Left face   (x = -1)  normal (-1,  0,  0) */
        m->vertices[20] = (struct vertex){ .pos = { .a = {-1, -1, -1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[21] = (struct vertex){ .pos = { .a = {-1, -1,  1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[22] = (struct vertex){ .pos = { .a = {-1,  1,  1} }, .normal = { .a = {-1,  0,  0} } };
        m->vertices[23] = (struct vertex){ .pos = { .a = {-1,  1, -1} }, .normal = { .a = {-1,  0,  0} } };

        /* Front  */
        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 2;
        m->indices[3] = 0; m->indices[4] = 2; m->indices[5] = 3;

        /* Back   */
        m->indices[6] = 4; m->indices[7] = 5; m->indices[8] = 6;
        m->indices[9] = 4; m->indices[10] = 6; m->indices[11] = 7;

        /* Top    */
        m->indices[12] = 8; m->indices[13] = 9; m->indices[14] = 10;
        m->indices[15] = 8; m->indices[16] = 10; m->indices[17] = 11;

       /* Bottom */
        m->indices[18] = 12; m->indices[19] = 13; m->indices[20] = 14;
        m->indices[21] = 12; m->indices[22] = 14; m->indices[23] = 15;

       /* Right  */
        m->indices[24] = 16; m->indices[25] = 17; m->indices[26] = 18;
        m->indices[27] = 16; m->indices[28] = 18; m->indices[29] = 19;

        /* Left   */
        m->indices[30] = 20; m->indices[31] = 21; m->indices[32] = 22;
        m->indices[33] = 20; m->indices[34] = 22; m->indices[35] = 23;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }
}


void init_plane(struct mesh* m, float w, float h, int tilt)
{
        float y = tilt ? -1.0f : 0.0f;

        m->vertex_count = 4;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 6;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->inward_normals = malloc(m->index_count * sizeof(struct vec3));
        m->restitution = 0.7f;
        m->static_mu = 0.15f;
        m->dynamic_mu = 0.1f;

        if (m->vertices == NULL ||
            m->indices == NULL ||
            m->inward_normals == NULL)
        {
                free(m->vertices);
                free(m->indices);
                free(m->inward_normals);

                return;
        }

        w = w / 2.0f;
        h = h / 2.0f;

        m->vertices[0] = (struct vertex){ .pos = { .a = {-w,  y, -h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[1] = (struct vertex){ .pos = { .a = {-w,  y,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[2] = (struct vertex){ .pos = { .a = { w,  0,  h} }, .normal = { .a = { 0,  1,  0} } };
        m->vertices[3] = (struct vertex){ .pos = { .a = { w,  0, -h} }, .normal = { .a = { 0,  1,  0} } };

        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 3;
        m->indices[3] = 1; m->indices[4] = 2; m->indices[5] = 3;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }

        mesh_normalize(m);
        mesh_inward_normalize(m);
}

void init_vert_plane(struct mesh* m, float x)
{
        m->vertex_count = 4;
        m->vertices = malloc(m->vertex_count * sizeof(struct vertex));
        m->index_count = 6;
        m->indices = malloc(m->index_count * sizeof(uint16_t));
        m->inward_normals = malloc(m->index_count * sizeof(struct vec3));
        m->restitution = 0.7f;
        m->static_mu = 0.15f;
        m->dynamic_mu = 0.1f;

        if (m->vertices == NULL ||
            m->indices == NULL ||
            m->inward_normals == NULL)
        {
                free(m->vertices);
                free(m->indices);
                free(m->inward_normals);

                return;
        }


        float w = 5.0f;
        float h = 5.0f;

        m->vertices[0] = (struct vertex){ .pos = { .a = {x,  h, -w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[1] = (struct vertex){ .pos = { .a = {x,  h,  w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[2] = (struct vertex){ .pos = { .a = {x,  -h,  w} }, .normal = { .a = { 1, 0, 0} } };
        m->vertices[3] = (struct vertex){ .pos = { .a = {x,  -h, -w} }, .normal = { .a = { 1, 0, 0} } };

        m->indices[0] = 0; m->indices[1] = 1; m->indices[2] = 3;
        m->indices[3] = 1; m->indices[4] = 2; m->indices[5] = 3;

        /* Colorize each vertex with a unique hue */
        for (int i = 0; i < m->vertex_count; i++) {
                float hue = (float)i / (float)m->vertex_count * 360.0f;
                float s = 0.8f, v = 0.9f;
                float c = v * s;
                float x = c * (1.0f - fabsf(fmodf(hue / 60.0f, 2.0f) - 1.0f));
                float mm = v - c;
                float r, g, b;
                if      (hue < 60)  { r = c; g = x; b = 0; }
                else if (hue < 120) { r = x; g = c; b = 0; }
                else if (hue < 180) { r = 0; g = c; b = x; }
                else if (hue < 240) { r = 0; g = x; b = c; }
                else if (hue < 300) { r = x; g = 0; b = c; }
                else                { r = c; g = 0; b = x; }
                m->vertices[i].color = (struct vec4){ .a = { r + mm, g + mm, b + mm, 1.0f } };
        }

        mesh_normalize(m);
        mesh_inward_normalize(m);
}
