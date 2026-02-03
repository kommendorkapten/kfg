#include <stdio.h>
#include <stdlib.h>
#include <pthread.h>
#include <stdint.h>
#include <sched.h>
#include <time.h>
#include <math.h>
#include <string.h>
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
};

struct world
{
        struct vec3 g;
        float dt;
        float air_density;
        struct mesh* surfaces;
        int num_surfaces;
        // threshod for squared velocity to considered to be in a steady state
        float ss_thr;
        // treshold for squared velocity during collitions
        float ss_c_thr;
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
        float restitution;
};

void set_high_priority(void);
void hybrid_sleep(unsigned long ns);
void update_objects(int, struct world*, struct object*, int, char);
void update_object(int step, struct world* w, struct object* o);
void init_object(struct object*);
void print_particle(const struct particle*);
void quantize_force(struct vec3*);
void drag_force(struct vec3*, const struct world*, const struct object*);
float vec3_dot(const struct vec3*, const struct vec3*);
struct vec3 vec3_add(const struct vec3*, const struct vec3*);
struct vec3 vec3_sub(const struct vec3*, const struct vec3*);
struct vec3 vec3_scalarm(const struct vec3* restrict, float);
/**
 * Normalize a vector
 * @param v the vector be to normalize
 * @return the noralized vector
 */
struct vec3 vec3_norm(const struct vec3*);
struct vec3 vec3_cross(const struct vec3* restrict,
                       const struct vec3* restrict);
void mesh_get_tri(struct vec3** restrict,
                  struct vec3** restrict,
                  struct vec3** restrict,
                  const struct mesh*,
                  int);
float math_rsqrt(float x);

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
 * @param v0 the first vertex of the triangle (CCW)
 * @param v1 the second vertex of the triangle
 * @param v2 the third vertex of the triangle
 * @param rc the combined restitution coefficent (prefer geometric mean)
 */
void collide_particle(struct particle* p,
                      struct vec3* restrict v0,
                      struct vec3* restrict v1,
                      struct vec3* restrict v2,
                      float rc);

int debug = 1;

int main(int argc, char** argv)
{
        int run = 1;
        int step = 0;
        int stop = 25 * FREQ;
        long t1;
        struct timing start;
        struct world w;
        struct object objs[NUM_OBJS];

        (void)argc;
        (void)argv;

        init_object(&objs[0]);
        objs[0].p.p.y = 5.0f;
        objs[0].m = 1.0f;
        objs[0].area = 0.3f;
        objs[0].drag_c = 0.47;
        objs[0].restitution = 0.9;
        print_particle(&objs[0].p);

        // create a mesh that (-1,1) (-1,-1) (1, -1) (1,1) in the x-z plane
        struct mesh s1;
        s1.vertices = malloc(4 * sizeof(struct vec3));
        s1.indices = malloc(6 * sizeof(int));
        s1.vertex_count = 4;
        s1.index_count = 6;
        s1.restitution = 1.0f;

        s1.vertices[0] = (struct vec3){-1.0f, 0.0f, -1.0f};
        s1.vertices[1] = (struct vec3){-1.0f, 0.0f, 1.0f};
        s1.vertices[2] = (struct vec3){1.0f, 0.0f, 1.0f};
        s1.vertices[3] = (struct vec3){1.0f, 0.0f, -1.0f};

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
        w.ss_thr   = 0.008 * 0.008; // 8mm/s
        w.ss_c_thr = 0.08 * 0.08; // 8cm/s

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

                if (debug)
                {
                        long now = timing_current_millis();

                        if (now - t1 > 1000)
                        {
                                t1 = now;

                                printf("%f seconds\n", (step + 1.0f)/ FREQ);
                                print = 1;
                        }
                }
                update_objects(step, &w, objs, NUM_OBJS, print);

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
        int ret = pthread_setschedparam(pthread_self(), SCHED_FIFO, &param);
        if (ret)
        {
                printf("failed to set scheduler: %s\n",
                       strerror(ret));
        }
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

void update_objects(int step,
                    struct world* w,
                    struct object* objs,
                    int n,
                    char print)
{
        for (int i = 0; i < n; i++)
        {
                struct object* o = objs + i;

                if (o->steady_state)
                {
                        continue;
                }

                update_object(step, w, o);

                if (print)
                {
                        print_particle(&o->p);
                }
        }
}

void update_object(int step, struct world* w, struct object* o)
{
        struct particle p;
        struct vec3 new_pos;
        struct vec3 f;
        int coll = 0;

        (void)step;

        // velocity half step (Verlet integration)
        o->p.v.x += o->p.a.x * w->dt * 0.5f;
        o->p.v.y += o->p.a.y * w->dt * 0.5f;
        o->p.v.z += o->p.a.z * w->dt * 0.5f;

        // position
        new_pos.x = o->p.p.x + o->p.v.x * w->dt;
        new_pos.y = o->p.p.y + o->p.v.y * w->dt;
        new_pos.z = o->p.p.z + o->p.v.z * w->dt;

        // Copy old pos
        // Compute the collision against a particle with the
        // original position and velocity set to the step's
        // velocity, i.e new_pos - old_pos.
        p.p = o->p.p;
        p.v = vec3_sub(&new_pos, &p.p);

        // collision detection
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
                        int coll_test;

                        mesh_get_tri(&v0,
                                     &v1,
                                     &v2,
                                     m,
                                     ti);

                        coll_test = ray_tri_intersect(&p,
                                                      v0,
                                                      v1,
                                                      v2,
                                                      &t,
                                                      &u,
                                                      &v);
                        /*
                          if a collision happens, it's measured in
                          units of the length, which is the current
                          step's computed velocity. The point of
                          intersection must be in this step
                        */
                        if (coll_test && t <= 1.0f)
                        {
                                float rc = sqrtf(m->restitution *
                                                 o->restitution);

                                coll = 1;
                                collide_particle(&o->p,
                                                 v0,
                                                 v1,
                                                 v2,
                                                 rc);

                                // is the object at rest?
                                float vabs = vec3_dot(&o->p.v, &o->p.v);

                                if (vabs < w->ss_c_thr)
                                {
                                        o->steady_state = 1;
                                }
                                break;
                        }
                }
                if (coll)
                {
                        break;
                }
        }
        if (!coll)
        {
                // Update position if there was no collision
                o->p.p = new_pos;
        }

        // compute all forces
        // gravity
        f.x = o->m * w->g.x;
        f.y = o->m * w->g.y;
        f.z = o->m * w->g.z;

        drag_force(&f, w, o);

        // compute new accelerations
        o->p.a.x = f.x / o->m;
        o->p.a.y = f.y / o->m;
        o->p.a.z = f.z / o->m;

        // velocity take two
        o->p.v.x += o->p.a.x * w->dt * 0.5f;
        o->p.v.y += o->p.a.y * w->dt * 0.5f;
        o->p.v.z += o->p.a.z * w->dt * 0.5f;

}

void collide_particle(struct particle* p,
                      struct vec3* restrict v0,
                      struct vec3* restrict v1,
                      struct vec3* restrict v2,
                      float rc)
{
        struct vec3 e1 = vec3_sub(v1, v0);
        struct vec3 e2 = vec3_sub(v2, v0);
        struct vec3 n = vec3_cross(&e1, &e2);

        n = vec3_norm(&n);
        float vn = vec3_dot(&p->v, &n);

        if (vn > 0.0)
        {
                printf("p.p %f %f %f\n", p->p.x, p->p.y, p->p.z);
                printf("p.v %f %f %f\n", p->v.x, p->v.y, p->v.z);
                printf("n %f %f %f\n", n.x, n.y, n.z);
                printf("v0 %f %f %f\n", v0->x, v0->y, v0->z);
                printf("v1 %f %f %f\n", v1->x, v1->y, v1->z);
                printf("v2 %f %f %f\n", v2->x, v2->y, v2->z);

                // object is moving away from the surface
                // should never happen
                printf("WARNING: collision when moving away from a surface\n");
                return;
        }

        // as n is normalized, vn is the velocity of the particle.
        // Compute the scaling factor with the restitution, and
        // reduce speed in the scaled normal's direction
        float factor = (1.0f + rc) * vn;
        n = vec3_scalarm(&n, factor);
        p->v = vec3_sub(&p->v, &n);
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
        o->steady_state = 0;
}

float vec3_dot(const struct vec3* v0,
               const struct vec3* v1)
{
        return v0->x * v1->x + v0->y * v1->y + v0->z * v1->z;
}

struct vec3 vec3_cross(const struct vec3* restrict v0,
                       const struct vec3* restrict v1)
{
        struct vec3  r;

        r.x = v0->y * v1->z - v0->z * v1->y;
        r.y = v0->z * v1->x - v0->x * v1->z;
        r.z = v0->x * v1->y - v0->y * v1->x;

        return r;
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
        float f;
        float inv_f;

        e1 = vec3_sub(v1, v0);
        e2 = vec3_sub(v2, v0);
        p = vec3_cross(&r->v, &e2);

        f = vec3_dot(&e1, &p);
        if (fabs(f) < epsilon)
        {
                // Ray is parallel to the triangle's plane
                return 0;
        }

        inv_f = 1.0f / f;
        s = vec3_sub(&r->p, v0);
        *u = inv_f * vec3_dot(&s, &p);
        if (*u < 0.0f || *u > 1.0f)
        {
                return 0;
        }

        q = vec3_cross(&s, &e1);
        *v = inv_f * vec3_dot(&r->v, &q);
        if (*v < 0.0f || *u + *v > 1.0f)
        {
                return 0;
        }

        *t = inv_f * vec3_dot(&e2, &q);
        if (*t < epsilon)
        {
                // particle is already behind triangle
                return 0;
        }

        return 1;
}

struct vec3 vec3_add(const struct vec3* v0,
                     const struct vec3* v1)
{
        struct vec3 r;

        r.x = v0->x + v1->x;
        r.y = v0->y + v1->y;
        r.z = v0->z + v1->z;

        return r;
}

struct vec3 vec3_sub(const struct vec3* v0,
                     const struct vec3* v1)
{
        struct vec3 r;

        r.x = v0->x - v1->x;
        r.y = v0->y - v1->y;
        r.z = v0->z - v1->z;

        return r;
}

struct vec3 vec3_scalarm(const struct vec3* restrict v, float s)
{
        struct vec3 r;

        r.x = v->x * s;
        r.y = v->y * s;
        r.z = v->z * s;

        return r;
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

float math_rsqrt(float x)
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

struct vec3 vec3_norm(const struct vec3* v)
{
        float rsq = math_rsqrt(vec3_dot(v, v));
        return vec3_scalarm(v, rsq);
}
