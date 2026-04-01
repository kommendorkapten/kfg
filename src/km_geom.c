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
#include <stdio.h>
#include <math.h>
#include <string.h>
#include "km_geom.h"
#include "km_phys.h"
#include "km_plat.h"
#include "../lib/cJSON.h"

void print_vertex(const struct vertex* v)
{
        printf("posi: %f %f %f\n", v->pos.x, v->pos.y, v->pos.z);
        printf("norm: %f %f %f\n", v->normal.x, v->normal.y, v->normal.z);
        printf("colr: %f %f %f %f\n", v->color.x, v->color.y,
               v->color.z, v->color.w);
}

void mesh_get_tri(struct vertex** restrict v0,
                  struct vertex** restrict v1,
                  struct vertex** restrict v2,
                  const struct mesh* m,
                  int i)
{
        *v0 = m->vertices + m->indices[i * 3 + 0];
        *v1 = m->vertices + m->indices[i * 3 + 1];
        *v2 = m->vertices + m->indices[i * 3 + 2];
}

int ray_tri_intersect(const struct particle* r,
                      const struct vec3* v0,
                      const struct vec3* v1,
                      const struct vec3* v2,
                      float* t,
                      float* u,
                      float* v)
{
        const float epsilon = 1e-6f;
        struct vec3 e1;
        struct vec3 e2;
        struct vec3 p;
        struct vec3 s;
        struct vec3 q;
        float f;
        float inv_f;

        e1 = vec3_sub(*v1, *v0);
        e2 = vec3_sub(*v2, *v0);
        p = vec3_cross(r->v, e2);

        f = vec3_dot(e1, p);
        if (fabs(f) < epsilon)
        {
                // Ray is parallel to the triangle's plane
                return 0;
        }

        inv_f = 1.0f / f;
        s = vec3_sub(r->p, *v0);
        *u = inv_f * vec3_dot(s, p);
        if (*u < 0.0f || *u > 1.0f)
        {
                return 0;
        }

        q = vec3_cross(s, e1);
        *v = inv_f * vec3_dot(r->v, q);
        if (*v < 0.0f || *u + *v > 1.0f)
        {
                return 0;
        }

        *t = inv_f * vec3_dot(e2, q);
        if (*t < epsilon)
        {
                // particle is already behind triangle
                return 0;
        }

        return 1;
}

int compute_toi(struct collision* toi,
                struct particle* p,
                struct mesh* meshes,
                int mesh_count)
{
        int coll_test;
        int ret = 0;

        toi->t = INFINITY;

        for (int s = 0; s < mesh_count; s++)
        {
                struct mesh* cm = meshes + s;
                uint16_t num_tri = cm->index_count / 3;

                for (uint16_t ti = 0; ti < num_tri; ti++)
                {
                        struct vec3 e1;
                        struct vec3 e2;

                        struct vertex* v0;
                        struct vertex* v1;
                        struct vertex* v2;
                        float t;
                        float u;
                        float v;

                        mesh_get_tri(&v0,
                                     &v1,
                                     &v2,
                                     cm,
                                     ti);

                        coll_test = ray_tri_intersect(p,
                                                      &v0->pos,
                                                      &v1->pos,
                                                      &v2->pos,
                                                      &t,
                                                      &u,
                                                      &v);

                        if (coll_test)
                        {
                                if (t < toi->t)
                                {
                                        e1 = vec3_sub(v1->pos, v0->pos);
                                        e2 = vec3_sub(v2->pos, v0->pos);
                                        toi->n = vec3_norm(vec3_cross(e1, e2));
                                        toi->t = t;
                                        toi->m = cm;
                                        toi->ti = ti;
                                        ret = 1;
                                }
                        }
                }
        }

        return ret;
}

void mesh_free(struct mesh* m)
{
        free(m->vertices);
        free(m->indices);
        memset(m, 0, sizeof(*m));
}

static char* read_file(const char* path)
{
        FILE* f = fopen(path, "rb");
        if (!f)
        {
                return NULL;
        }

        fseek(f, 0, SEEK_END);
        long len = ftell(f);
        fseek(f, 0, SEEK_SET);

        char* buf = malloc((unsigned long)len + 1);
        if (!buf)
        {
                fclose(f);
                return NULL;
        }

        size_t nr = fread(buf, 1, (unsigned long)len, f);
        fclose(f);

        if ((long)nr != len)
        {
                free(buf);
                return NULL;
        }

        buf[len] = '\0';
        return buf;
}

static struct mesh* parse_mesh(const cJSON* json_mesh)
{
        struct mesh* m = malloc(sizeof(*m));
        if (!m)
        {
                return NULL;
        }
        memset(m, 0, sizeof(*m));
        m->static_mu = 0.5f;
        m->dynamic_mu = 0.5f;

        /* restitution */
        cJSON* rest = cJSON_GetObjectItem(json_mesh, "restitution");
        if (cJSON_IsNumber(rest))
        {
                m->restitution = (float)rest->valuedouble;
        }

        /* friction coefficients (optional) */
        cJSON* smu = cJSON_GetObjectItem(json_mesh, "static_mu");
        if (cJSON_IsNumber(smu))
        {
                m->static_mu = (float)smu->valuedouble;
        }
        cJSON* dmu = cJSON_GetObjectItem(json_mesh, "dynamic_mu");
        if (cJSON_IsNumber(dmu))
        {
                m->dynamic_mu = (float)dmu->valuedouble;
        }

        /* grid dimensions (optional) */
        cJSON* gx = cJSON_GetObjectItem(json_mesh, "grid_x");
        if (cJSON_IsNumber(gx))
        {
                m->grid_x = (uint16_t)gx->valuedouble;
        }
        cJSON* gz = cJSON_GetObjectItem(json_mesh, "grid_z");
        if (cJSON_IsNumber(gz))
        {
                m->grid_z = (uint16_t)gz->valuedouble;
        }

        /* vertices */
        cJSON* verts = cJSON_GetObjectItem(json_mesh, "vertices");
        if (!cJSON_IsArray(verts))
        {
                printf("failed to get vertices\n");
                free(m);
                return NULL;
        }

        int vc = cJSON_GetArraySize(verts);
        m->vertex_count = (uint16_t)vc;
        m->vertices = malloc((unsigned long)vc * sizeof(struct vertex));
        if (!m->vertices)
        {
                free(m);
                return NULL;
        }
#ifdef DEBUG
        printf("found %d vertices\n", vc);
#endif

        for (int i = 0; i < vc; i++)
        {
                cJSON* v = cJSON_GetArrayItem(verts, i);
                if (!cJSON_IsObject(v))
                {
                        fprintf(stderr, "vertex %d: expected object\n", i);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }

                cJSON* pos = cJSON_GetObjectItem(v, "position");
                if (!cJSON_IsArray(pos) || cJSON_GetArraySize(pos) < 3)
                {
                        fprintf(stderr, "vertex %d: 'position' must be an "
                                "array with at least 3 elements\n", i);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }

                cJSON* norm = cJSON_GetObjectItem(v, "normal");
                if (!cJSON_IsArray(norm) || cJSON_GetArraySize(norm) < 3)
                {
                        fprintf(stderr, "vertex %d: 'normal' must be an "
                                "array with at least 3 elements\n", i);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }

                cJSON* px = cJSON_GetArrayItem(pos, 0);
                cJSON* py = cJSON_GetArrayItem(pos, 1);
                cJSON* pz = cJSON_GetArrayItem(pos, 2);
                if (!cJSON_IsNumber(px) || !cJSON_IsNumber(py) ||
                    !cJSON_IsNumber(pz))
                {
                        fprintf(stderr, "vertex %d: 'position' elements "
                                "must be numbers\n", i);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }

                cJSON* nx = cJSON_GetArrayItem(norm, 0);
                cJSON* ny = cJSON_GetArrayItem(norm, 1);
                cJSON* nz = cJSON_GetArrayItem(norm, 2);
                if (!cJSON_IsNumber(nx) || !cJSON_IsNumber(ny) ||
                    !cJSON_IsNumber(nz))
                {
                        fprintf(stderr, "vertex %d: 'normal' elements "
                                "must be numbers\n", i);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }

                m->vertices[i].pos.x = (float)px->valuedouble;
                m->vertices[i].pos.y = (float)py->valuedouble;
                m->vertices[i].pos.z = (float)pz->valuedouble;

                m->vertices[i].normal.x = (float)nx->valuedouble;
                m->vertices[i].normal.y = (float)ny->valuedouble;
                m->vertices[i].normal.z = (float)nz->valuedouble;

                /* color (optional, default: 0.3, 0.6, 0.8, 1.0) */
                cJSON* col = cJSON_GetObjectItem(v, "color");
                if (cJSON_IsArray(col) && cJSON_GetArraySize(col) >= 4)
                {
                        cJSON* cr = cJSON_GetArrayItem(col, 0);
                        cJSON* cg = cJSON_GetArrayItem(col, 1);
                        cJSON* cb = cJSON_GetArrayItem(col, 2);
                        cJSON* ca = cJSON_GetArrayItem(col, 3);
                        if (cJSON_IsNumber(cr) && cJSON_IsNumber(cg) &&
                            cJSON_IsNumber(cb) && cJSON_IsNumber(ca))
                        {
                                m->vertices[i].color.x = (float)cr->valuedouble;
                                m->vertices[i].color.y = (float)cg->valuedouble;
                                m->vertices[i].color.z = (float)cb->valuedouble;
                                m->vertices[i].color.w = (float)ca->valuedouble;
                        }
                        else
                        {
                                m->vertices[i].color = (struct vec4)
                                        {.x = 0.3f, .y = 0.6f,
                                         .z = 0.8f, .w = 1.0f};
                        }
                }
                else
                {
                        m->vertices[i].color = (struct vec4)
                                {.x = 0.3f, .y = 0.6f,
                                 .z = 0.8f, .w = 1.0f};
                }
#ifdef DEBUG
                printf("vertex %d\n", i);
                print_vertex(m->vertices + i);
#endif
        }

        /* indices */
        cJSON* idxs = cJSON_GetObjectItem(json_mesh, "indices");
        if (!cJSON_IsArray(idxs))
        {
                free(m->vertices);
                free(m);
                printf("failed to get indices\n");
                return NULL;
        }

        int ic = cJSON_GetArraySize(idxs);
        m->index_count = (uint16_t)ic;
        m->indices = malloc((unsigned long)ic * sizeof(uint16_t));
        if (!m->indices)
        {
                free(m->vertices);
                free(m);
                return NULL;
        }
#ifdef DEBUG
        printf("found %d indices\n", ic);
#endif

        for (int i = 0; i < ic; i++)
        {
                cJSON* idx = cJSON_GetArrayItem(idxs, i);
                if (!cJSON_IsNumber(idx))
                {
                        fprintf(stderr, "index %d: expected a number\n", i);
                        free(m->indices);
                        free(m->vertices);
                        free(m);
                        return NULL;
                }
                m->indices[i] = (uint16_t)idx->valuedouble;
        }

        return m;
}

struct mesh* load_meshes(const char* p, int* count)
{
        *count = 0;

        char* data = read_file(p);
        if (!data)
        {
                printf("failed to parse read file\n");
                return NULL;
        }

        cJSON* root = cJSON_Parse(data);
        free(data);
        if (!root)
        {
                printf("failed to parse json data\n");
                return NULL;
        }

        cJSON* meshes = cJSON_GetObjectItem(root, "meshes");
        if (!cJSON_IsArray(meshes))
        {
                printf("failed to get meshes\n");
                cJSON_Delete(root);
                return NULL;
        }

        size_t n = (size_t)cJSON_GetArraySize(meshes);
        if (n == 0)
        {
                printf("failed to get array size\n");
                cJSON_Delete(root);
                return NULL;
        }

        if (n > UINT16_MAX)
        {
                printf("too many meshes found: %zu\n", n);
                cJSON_Delete(root);
                return NULL;
        }

#ifdef DEBUG
        printf("found %zu meshes\n", n);
#endif

        /* Parse each mesh into its own allocation */
        struct mesh** tmp = malloc(n * sizeof(struct mesh*));
        if (!tmp)
        {
                cJSON_Delete(root);
                return NULL;
        }

        int parsed = 0;
        for (int i = 0; i < (int)n; i++)
        {
                cJSON* item = cJSON_GetArrayItem(meshes, i);
                tmp[i] = parse_mesh(item);
                if (!tmp[i])
                {
                        /* Clean up previously parsed meshes */
                        for (int j = 0; j < parsed; j++)
                        {
                                mesh_free(tmp[j]);
                                free(tmp[j]);
                        }
                        free(tmp);
                        cJSON_Delete(root);
                        return NULL;
                }
                parsed++;
        }

        cJSON_Delete(root);

        /* Consolidate into a single contiguous array */
        struct mesh* result = malloc(n * sizeof(struct mesh));
        if (!result)
        {
                for (int i = 0; i < (int)n; i++)
                {
                        mesh_free(tmp[i]);
                        free(tmp[i]);
                }
                free(tmp);
                return NULL;
        }

        for (int i = 0; i < (int)n; i++)
        {
                result[i] = *tmp[i];
                free(tmp[i]);
        }
        free(tmp);

        *count = (int)n;
        return result;
}

int write_meshes(const char* p, const struct mesh* meshes, int count)
{
        cJSON* root = cJSON_CreateObject();
        if (!root)
        {
                return -1;
        }

        cJSON* jarr = cJSON_AddArrayToObject(root, "meshes");
        if (!jarr)
        {
                cJSON_Delete(root);
                return -1;
        }

        for (int i = 0; i < count; i++)
        {
                const struct mesh* m = &meshes[i];
                cJSON* jmesh = cJSON_CreateObject();
                if (!jmesh)
                {
                        cJSON_Delete(root);
                        return -1;
                }

                cJSON_AddNumberToObject(jmesh, "restitution", m->restitution);
                cJSON_AddNumberToObject(jmesh, "static_mu", m->static_mu);
                cJSON_AddNumberToObject(jmesh, "dynamic_mu", m->dynamic_mu);

                if (m->grid_x > 0 && m->grid_z > 0)
                {
                        cJSON_AddNumberToObject(jmesh, "grid_x", m->grid_x);
                        cJSON_AddNumberToObject(jmesh, "grid_z", m->grid_z);
                }

                /* vertices */
                cJSON* jverts = cJSON_AddArrayToObject(jmesh, "vertices");
                for (int vi = 0; vi < m->vertex_count; vi++)
                {
                        const struct vertex* v = &m->vertices[vi];
                        cJSON* jv = cJSON_CreateObject();

                        cJSON* pos = cJSON_CreateFloatArray(v->pos.a, 3);
                        cJSON_AddItemToObject(jv, "position", pos);

                        cJSON* norm = cJSON_CreateFloatArray(v->normal.a, 3);
                        cJSON_AddItemToObject(jv, "normal", norm);

                        cJSON* col = cJSON_CreateFloatArray(v->color.a, 4);
                        cJSON_AddItemToObject(jv, "color", col);

                        cJSON_AddItemToArray(jverts, jv);
                }

                /* indices */
                cJSON* jidxs = cJSON_AddArrayToObject(jmesh, "indices");
                for (int ii = 0; ii < m->index_count; ii++)
                {
                        cJSON_AddItemToArray(jidxs,
                                cJSON_CreateNumber(m->indices[ii]));
                }

                cJSON_AddItemToArray(jarr, jmesh);
        }

        char* json_str = cJSON_Print(root);
        cJSON_Delete(root);
        if (!json_str)
        {
                return -1;
        }

        FILE* f = fopen(p, "w");
        if (!f)
        {
                free(json_str);
                return -1;
        }

        fputs(json_str, f);
        fclose(f);
        free(json_str);

        return 0;
}

struct mesh* gen_mesh(float x, float y, float d)
{
        int count_x = (int)(x / d) + 1;
        int count_y = (int)(y / d) + 1;

        if (count_x > UINT16_MAX || count_y > UINT16_MAX)
        {
                fprintf(stderr, "too large mesh (%d x %d)\n",
                        count_x, count_y);
                return NULL;
        }
        struct mesh* m = malloc(sizeof(*m));
        struct vertex* v;
        int v_count = count_x * count_y;
        int q_count = (count_x - 1) * (count_y - 1);
        int t_count = q_count * 2;
        int i_count = t_count * 3;
        int ip = 0;

        if (!m)
        {
                return NULL;
        }

        if (v_count > UINT16_MAX || i_count > UINT16_MAX)
        {
                free(m);
                fprintf(stderr, "too large mesh v count %d idx count %d\n",
                        v_count, i_count);
                return NULL;
        }

        m->grid_x = (uint16_t)count_x;
        m->grid_z = (uint16_t)count_y;

#ifdef DEBUG
        printf("x: %d y: %d\n", count_x, count_y);
        printf("num quads %d num vert: %d num tri %d num indi %d\n",
               q_count, v_count, t_count, i_count);
#endif

        if (d > x || d > y)
        {
                printf("invalid %f x %f y %f\n", d, x, y);
                free(m);
                return NULL;
        }

        m->vertices = malloc((unsigned long)v_count * sizeof(struct vertex));
        m->indices = malloc((unsigned long)i_count * sizeof(uint16_t));
        m->vertex_count = (uint16_t)v_count;
        m->index_count = (uint16_t)i_count;

        for (int iy = 0; iy < count_y; iy++)
        {
                for (int ix = 0; ix < count_x; ix++)
                {
                        v = m->vertices + ip;

                        v->pos.x = (float)ix * d;
                        v->pos.y = 0.0f;
                        v->pos.z = (float)iy * d;

                        v->color.x = 0.2f;
                        v->color.y = 0.2f;
                        v->color.z = 0.7f;
                        v->color.w = 1.0f;

#ifdef DEBUG
                        printf("%d %f %f\n", ip, v->pos.x, v->pos.z);
#endif
                        ip++;
                }
        }

        ip = 0;
        // Iterate through each quad
        for (int iy = 0; iy < count_y - 1; iy++)
        {
                for (int ix = 0; ix < count_x - 1; ix++)
                {
                        // first triangle
                        m->indices[ip++] = (uint16_t)(iy * count_x +ix);
                        m->indices[ip++] = (uint16_t)((iy + 1) * count_x + ix);
                        m->indices[ip++] = (uint16_t)(iy * count_x + 1 + ix);

                        // second triangle
                        m->indices[ip++] = (uint16_t)(iy * count_x + 1 + ix);
                        m->indices[ip++] = (uint16_t)((iy + 1) * count_x + ix);
                        m->indices[ip++] = (uint16_t)((iy + 1) * count_x + 1 + ix);

#ifdef DEBUG
                        printf("%d %d (%d %d %d) (%d %d %d)\n",
                               ix, iy,
                               m->indices[ip-6],
                               m->indices[ip-5],
                               m->indices[ip-4],
                               m->indices[ip-3],
                               m->indices[ip-2],
                               m->indices[ip-1]);
#endif
                }
        }

        return m;
}

void mesh_heightmap(struct mesh* m, int peaks, float max_height, float radius)
{
        /* Find mesh extents in xz plane */
        float min_x = m->vertices[0].pos.x;
        float max_x = min_x;
        float min_z = m->vertices[0].pos.z;
        float max_z = min_z;

        for (uint16_t i = 1; i < m->vertex_count; i++)
        {
                float x = m->vertices[i].pos.x;
                float z = m->vertices[i].pos.z;
                if (x < min_x) min_x = x;
                if (x > max_x) max_x = x;
                if (z < min_z) min_z = z;
                if (z > max_z) max_z = z;
        }

        float range_x = max_x - min_x;
        float range_z = max_z - min_z;

        /* Generate random peak positions and heights */
        float* peak_x = malloc((unsigned long)peaks * sizeof(float));
        float* peak_z = malloc((unsigned long)peaks * sizeof(float));
        float* peak_h = malloc((unsigned long)peaks * sizeof(float));

        for (int i = 0; i < peaks; i++)
        {
                peak_x[i] = min_x + range_x * rand_u01();
                peak_z[i] = min_z + range_z * rand_u01();
                peak_h[i] = max_height * rand_u01();
        }

        /* For each vertex, sum contributions from all peaks */
        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                float h = 0.0f;
                float vx = m->vertices[i].pos.x;
                float vz = m->vertices[i].pos.z;

                for (int p = 0; p < peaks; p++)
                {
                        float dx = vx - peak_x[p];
                        float dz = vz - peak_z[p];
                        float dist = sqrtf(dx * dx + dz * dz);
                        float falloff = 1.0f - dist / radius;

                        if (falloff > 0.0f)
                        {
                                h += peak_h[p] * falloff;
                        }
                }

                m->vertices[i].pos.y = h;
        }

        free(peak_x);
        free(peak_z);
        free(peak_h);
}

void mesh_normalize(struct mesh* m)
{
        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                m->vertices[i].normal = (struct vec3){ .a = {0.0f, 0.0f, 0.0f} };
        }

        // Iterate through all triangles, add the normal to each vertex
        // the cross product is proportional to the triangle's area, which
        // makes bigger triangles contribute more to the normal
        for (uint16_t i = 0; i < m->index_count / 3; i++)
        {
                struct vertex* v0;
                struct vertex* v1;
                struct vertex* v2;
                struct vec3 e1;
                struct vec3 e2;
                struct vec3 n;

                mesh_get_tri(&v0, &v1, &v2, m, i);

                e1 = vec3_sub(v1->pos, v0->pos);
                e2 = vec3_sub(v2->pos, v0->pos);

                n = vec3_cross(e1, e2);

                v0->normal = vec3_add(v0->normal, n);
                v1->normal = vec3_add(v1->normal, n);
                v2->normal = vec3_add(v2->normal, n);
        }

        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                m->vertices[i].normal = vec3_norm(m->vertices[i].normal);
        }
}

void mesh_translate(struct mesh* m, struct vec3* v)
{
        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                m->vertices[i].pos = vec3_add(m->vertices[i].pos, *v);
        }
}

void mesh_colorize(struct mesh* m)
{
        /* Find min/max height (y) */
        float min_y = m->vertices[0].pos.y;
        float max_y = min_y;

        for (uint16_t i = 1; i < m->vertex_count; i++)
        {
                float y = m->vertices[i].pos.y;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
        }

        float range = max_y - min_y;
        if (range < 1e-6f)
        {
                range = 1.0f;
        }

        /*
         * Color stops (normalized height 0..1):
         *   0.00  dark     (0.05, 0.05, 0.05)
         *   0.33  green    (0.15, 0.45, 0.10)
         *   0.66  brown    (0.45, 0.30, 0.15)
         *   1.00  sand     (0.82, 0.75, 0.55)
         */
        const float stops[] = { 0.0f, 0.33f, 0.66f, 1.0f };
        const float colors[][3] = {
                { 0.05f, 0.05f, 0.05f },
                { 0.15f, 0.45f, 0.10f },
                { 0.45f, 0.30f, 0.15f },
                { 0.82f, 0.75f, 0.55f },
        };
        const int n_stops = 4;

        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                float t = (m->vertices[i].pos.y - min_y) / range;

                /* Find which segment t falls into */
                int seg = n_stops - 2;
                for (int s = 0; s < n_stops - 1; s++)
                {
                        if (t <= stops[s + 1])
                        {
                                seg = s;
                                break;
                        }
                }

                /* Interpolate within the segment */
                float seg_range = stops[seg + 1] - stops[seg];
                float local_t = (t - stops[seg]) / seg_range;

                m->vertices[i].color.x = colors[seg][0] +
                        (colors[seg + 1][0] - colors[seg][0]) * local_t;
                m->vertices[i].color.y = colors[seg][1] +
                        (colors[seg + 1][1] - colors[seg][1]) * local_t;
                m->vertices[i].color.z = colors[seg][2] +
                        (colors[seg + 1][2] - colors[seg][2]) * local_t;
                m->vertices[i].color.w = 1.0f;
        }
}

void mesh_colorize_water(struct mesh* m)
{
        /* Find min/max height (y) */
        float min_y = m->vertices[0].pos.y;
        float max_y = min_y;

        for (uint16_t i = 1; i < m->vertex_count; i++)
        {
                float y = m->vertices[i].pos.y;
                if (y < min_y) min_y = y;
                if (y > max_y) max_y = y;
        }

        float range = max_y - min_y;
        if (range < 1e-6f)
        {
                range = 1.0f;
        }

        /*
         * Color stops (normalized height 0..1):
         *   0.00  deep blue   (0.02, 0.05, 0.20)
         *   0.50  ocean blue  (0.05, 0.20, 0.55)
         *   1.00  light cyan  (0.40, 0.70, 0.85)
         */
        const float stops[] = { 0.0f, 0.50f, 1.0f };
        const float colors[][3] = {
                { 0.02f, 0.05f, 0.20f },
                { 0.05f, 0.20f, 0.55f },
                { 0.40f, 0.70f, 0.85f },
        };
        const int n_stops = 3;

        for (uint16_t i = 0; i < m->vertex_count; i++)
        {
                float t = (m->vertices[i].pos.y - min_y) / range;

                int seg = n_stops - 2;
                for (int s = 0; s < n_stops - 1; s++)
                {
                        if (t <= stops[s + 1])
                        {
                                seg = s;
                                break;
                        }
                }

                float seg_range = stops[seg + 1] - stops[seg];
                float local_t = (t - stops[seg]) / seg_range;

                m->vertices[i].color.x = colors[seg][0] +
                        (colors[seg + 1][0] - colors[seg][0]) * local_t;
                m->vertices[i].color.y = colors[seg][1] +
                        (colors[seg + 1][1] - colors[seg][1]) * local_t;
                m->vertices[i].color.z = colors[seg][2] +
                        (colors[seg + 1][2] - colors[seg][2]) * local_t;
                m->vertices[i].color.w = 1.0f;
        }
}
