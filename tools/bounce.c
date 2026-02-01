#include <stdio.h>
#include <stdio.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <math.h>
#include "timing.h"

#define PERIOD   16666666
#define FREQ           60
#define SECOND 1000000000
#define NUM_OBJS 1

struct vec3
{
        float x;
        float y;
        float z;
};

struct particle
{
        struct vec3 p;
        struct vec3 v;
        struct vec3 a;
};

struct object
{
        struct particle p;
        float m;
        float area;
        float drag_c;
};

struct world
{
        struct vec3 g;
        float dt;
        float air_density;
        struct mesh* surfaces;
        int num_surfaces;
};

/*
  Access triangle i
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
};

void set_high_priority(void);
void hybrid_sleep(unsigned long ns);
void update_objects(struct world*, struct object*, int, char);
void init_object(struct object*);
void print_particle(const struct particle*);
void quantize_force(struct vec3*);
void drag_force(struct vec3*, const struct world*, const struct object*);
float vec3_dot(const struct vec3* restrict, const struct vec3* restrict);
void vec3_add(struct vec3* restrict, const struct vec3*, const struct vec3*);
void vec3_sub(struct vec3* restrict, const struct vec3*, const struct vec3*);
void vec3_scalarm(struct vec3* restrict, const struct vec3* restrict, float);
void vec3_cross(struct vec3* restrict,
            const struct vec3* restrict,
            const struct vec3* restrict);
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

int debug = 1;

int main(int argc, char** argv)
{
        int run = 1;
        int step = 0;
        int stop = 6 * FREQ;
        long t1;
        struct timing start;
        struct world w;
        struct object objs[NUM_OBJS];

        (void)argc;
        (void)argv;

        init_object(&objs[0]);
        objs[0].p.p.y = 5.0f;
        objs[0].m = 1.0f;
        objs[0].area = 1.0f;
        objs[0].drag_c = 0.47;
        print_particle(&objs[0].p);

        // create a mesh that (-1,1) (-1,-1) (1, -1) (1,1) in the x-z plane
        struct mesh s1;
        s1.vertices = malloc(4 * sizeof(struct vec3));
        s1.indices = malloc(6 * sizeof(int));
        s1.vertex_count = 4;
        s1.index_count = 6;

        s1.vertices[0] = (struct vec3){-1.0f, 0.0f, 1.0f};
        s1.vertices[1] = (struct vec3){-1.0f, 0.0f, -1.0f};
        s1.vertices[2] = (struct vec3){1.0f, 0.0f, -1.0f};
        s1.vertices[3] = (struct vec3){1.0f, 0.0f, 1.0f};

        s1.indices[0] = 0;
        s1.indices[1] = 1;
        s1.indices[2] = 2;
        s1.indices[3] = 2;
        s1.indices[4] = 3;
        s1.indices[5] = 0;

        w.g = (struct vec3){0.0f, -9.82f, 0.0f};
        w.dt = (float)((double)PERIOD/(double)SECOND);
        w.air_density = 1.225;
        w.surfaces = &s1;
        w.num_surfaces = 1;

        if (debug)
        {
                printf("Using dt %f\n", w.dt);
        }

        set_high_priority();

        timing_start(&start);
        t1 = timing_current_millis();
        while (run)
        {
                long begin = timing_current_usec();
                char print = 0;
                // debug
                if (debug)
                {
                        long now = timing_current_millis();

                        if (now - t1 > 1000)
                        {
                                t1 = now;

                                printf("%d seconds\n", (step + 1)/ FREQ);
                                print = 1;
                        }
                }

                update_objects(&w, objs, NUM_OBJS, print);

                // wait for next step
                long sbegin = timing_current_usec();
                hybrid_sleep(PERIOD - (sbegin - begin) * 1000);

                if (step >= stop) {
                        run = 0;
                }
                step++;
        }

        printf("ran for %ldms\n", timing_dur_msec(&start));

        print_particle(&objs[0].p);

        return 0;
}

void set_high_priority(void) {
        struct sched_param param = {
                .sched_priority = sched_get_priority_max(SCHED_FIFO)
        };
        pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
}

void hybrid_sleep(unsigned long ns)
{
        long start = timing_current_usec();
        long left;

        // sleep for 80% of the request time
        struct timespec sleep = {
                .tv_sec = 0,
                .tv_nsec = (long)((float)(ns) * 0.8f)
        };
        nanosleep(&sleep, NULL);

        left = ns / 1000 - (timing_current_usec() - start);
        while (left > 0) {
                left = ns / 1000 - (timing_current_usec() - start);
        }
}

void update_objects(struct world* w, struct object* objs, int n, char print)
{
        for (int i = 0; i < n; i++)
        {
                struct object* o = objs + i;
                struct vec3 f;

                // velocity half step (Verlet integration)
                o->p.v.x += o->p.a.x * w->dt * 0.5f;
                o->p.v.y += o->p.a.y * w->dt * 0.5f;
                o->p.v.z += o->p.a.z * w->dt * 0.5f;

                // position
                o->p.p.x += o->p.v.x * w->dt;
                o->p.p.y += o->p.v.y * w->dt;
                o->p.p.z += o->p.v.z * w->dt;

                // collision detection (including compute forces)
                for (int s = 0; s < w->num_surfaces; s++)
                {
                        struct mesh* m = w->surfaces + s;
                        int num_tri = m->index_count / 3;

                        for (int ti = 0; ti < num_tri; ti++)
                        {
                                struct vec3* v0;
                                struct vec3* v1;
                                struct vec3* v2;
                                float t;
                                float u;
                                float v;
                                int coll;

                                mesh_get_tri(&v0,
                                             &v1,
                                             &v2,
                                             m,
                                             ti);

                                coll = ray_tri_intersect(&o->p,
                                                         v0,
                                                         v1,
                                                         v2,
                                                         &t,
                                                         &u,
                                                         &v);
                                /*
                                  if a collision happens, it's measured in
                                  units of the length of the particle's
                                  velocity. However, we are only doing
                                  increments of v/dt, so we need to account
                                  for that.
                                */
                                if (coll && t < w->dt)
                                {
                                        printf("*** COLLISION *** %f\n", t);
                                }
                        }
                }

                // compute all forces
                // gravity
                f.x = o->m * w->g.x;
                f.y = o->m * w->g.y;
                f.z = o->m * w->g.z;

                drag_force(&f, w, o);
                quantize_force(&f);

                // compute new accelerations
                o->p.a.x = f.x / o->m;
                o->p.a.y = f.y / o->m;
                o->p.a.z = f.z / o->m;

                // velocity
                o->p.v.x += o->p.a.x * w->dt * 0.5f;
                o->p.v.y += o->p.a.y * w->dt * 0.5f;
                o->p.v.z += o->p.a.z * w->dt * 0.5f;

                if (print)
                {
                        print_particle(&o->p);
                        printf("for: %f %f %f\n", f.x, f.y, f.z);
                }
        }
}

void drag_force(struct vec3* f, const struct world* w, const struct object* o)
{
        float half_rho = w->air_density * 0.5f;
        float cd = o->area * o->drag_c;

        f->x -= half_rho * fabsf(o->p.v.x) * o->p.v.x * cd;
        f->y -= half_rho * fabsf(o->p.v.y) * o->p.v.y * cd;
        f->z -= half_rho * fabsf(o->p.v.z) * o->p.v.z * cd;
}

void quantize_force(struct vec3* f)
{
        float thr = 0.01;

        if (fabsf(f->x) < thr) f->x = 0.0f;
        if (fabsf(f->y) < thr) f->y = 0.0f;
        if (fabsf(f->z) < thr) f->z = 0.0f;
}

void print_particle(const struct particle* p)
{
        printf("pos: %f %f %f\n", p->p.x, p->p.y, p->p.z);
        printf("vel: %f %f %f\n", p->v.x, p->v.y, p->v.z);
        printf("acl: %f %f %f\n", p->a.x, p->a.y, p->a.z);
}

void init_object(struct object* o)
{
        o->p.p = (struct vec3){0.0f, 0.0f, 0.0f};
        o->p.v = (struct vec3){0.0f, 0.0f, 0.0f};
        o->p.a = (struct vec3){0.0f, 0.0f, 0.0f};
}

float vec3_dot(const struct vec3* restrict v0,
               const struct vec3* restrict v1)
{
        return v0->x * v1->x + v0->y * v1->y + v0->z * v1->z;
}

void vec3_cross(struct vec3* restrict r,
                const struct vec3* restrict v0,
                const struct vec3* restrict v1)
{
        r->x = v0->y * v1->z - v0->z * v1->y;
        r->y = v0->z * v1->x - v0->x * v1->z;
        r->z = v0->x * v1->y - v0->y * v1->x;
}

int ray_tri_intersect(const struct particle* r,
                      const struct vec3* v0,
                      const struct vec3* v1,
                      const struct vec3* v2,
                      float* t,
                      float* u,
                      float* v)
{
        const float epsilon = 1e-6;
        struct vec3 e1;
        struct vec3 e2;
        struct vec3 p;
        struct vec3 s;
        struct vec3 q;
        float det;
        float inv_det;

        vec3_sub(&e1, v1, v0);
        vec3_sub(&e2, v2, v0);
        vec3_cross(&p, &r->v, &e2);

        det = vec3_dot(&e1, &p);
        if (fabs(det) < epsilon)
        {
                return 0;
        }

        inv_det = 1.0f / det;
        vec3_sub(&s, &r->p, v0);
        *u = inv_det * vec3_dot(&s, &p);
        if (*u < 0.0f || *u > 1.0f)
        {
                return 0;
        }

        vec3_cross(&q, &s, &e1);
        *v = inv_det * vec3_dot(&r->v, &q);
        if (*v < 0.0f || *u + *v > 1.0f)
        {
                return 0;
        }

        *t = inv_det * vec3_dot(&e2, &q);
        if (*t < epsilon)
        {
                // particle is already behind triangle
                return 0;
        }

        return 1;
}

void vec3_add(struct vec3* restrict r,
              const struct vec3* v0,
              const struct vec3* v1)
{
        r->x = v0->x + v1->x;
        r->y = v0->y + v1->y;
        r->z = v0->z + v1->z;
}

void vec3_sub(struct vec3* restrict r,
              const struct vec3* v0,
              const struct vec3* v1)
{
        r->x = v0->x - v1->x;
        r->y = v0->y - v1->y;
        r->z = v0->z - v1->z;
}

void vec3_scalarm(struct vec3* restrict r,
                  const struct vec3* restrict v,
                  float s)
{
        r->x = v->x * s;
        r->y = v->y * s;
        r->z = v->z * s;
}

void mesh_get_tri(struct vec3** restrict v0,
                  struct vec3** restrict v1,
                  struct vec3** restrict v2,
                  const struct mesh* m,
                  int i)
{
        *v0 = m->vertices + m->indices[i * 3 + 0];
        *v1 = m->vertices + m->indices[i * 3 + 1];
        *v2 = m->vertices + m->indices[i * 3 + 2];
}
