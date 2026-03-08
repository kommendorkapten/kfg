#include <stdio.h>
#include <math.h>
#include <stdlib.h>

#include "km_phys.h"
#include "km_geom.h"

int test_sliding_friction(void)
{
        struct world w;
        struct mesh m;
        struct object o = {0};
        int ret = 0;
        int freq = 1000; // Using a high frequency to minimize integration error over the steps
        float v0 = 5.0f;
        
        default_world(&w, freq);
        w.surface_count = 1;
        w.surfaces = &m;
        
        // A large flat surface at y = 0
        m.vertex_count = 3;
        m.vertices = malloc(m.vertex_count * sizeof(struct vertex));
        m.vertices[0].pos = (struct vec3) { .a = { -10.0f, 0.0f, -10.0f } };
        m.vertices[1].pos = (struct vec3) { .a = { -10.0f, 0.0f, 100.0f } };
        m.vertices[2].pos = (struct vec3) { .a = { 100.0f, 0.0f, -10.0f } };
        m.index_count = 3;
        m.indices = malloc(m.index_count * sizeof(int));
        m.indices[0] = 0;
        m.indices[1] = 1;
        m.indices[2] = 2;
        m.restitution = 0.0f; // Don't bounce
        m.static_mu = 0.5f;
        m.dynamic_mu = 0.5f;

        o.p.v.x = v0;
        o.p.v.y = 0.0f;
        o.p.v.z = 0.0f;
        o.p.p.x = 0.0f;
        o.p.p.y = 0.001f; // Just above surface
        o.p.p.z = 0.0f;
        o.p.a.y = - KM_PHYS_G;
        o.m = 1.0f;
        o.area = 0.0f; // Isolate from air drag
        o.drag_c = 0.0f;
        o.restitution = 0.0f;
        o.static_mu = 0.5f;
        o.dynamic_mu = 0.5f;
        o.steady_state = 0;
        
        // Expected distance: d = v^2 / (2 * mu * g)
        // Note: mu = sqrt(obj_mu * mesh_mu) = 0.5
        float expected_d = (v0 * v0) / (2.0f * 0.5f * KM_PHYS_G);
        
        // Expected time: t = v / (mu * g)
        float expected_t = v0 / (0.5f * KM_PHYS_G);
        
        int max_steps = 10000;
        int step = 0;
        
        for (step = 0; step < max_steps; step++)
        {
                update_object(step, &w, &o);
                if (step % 50 == 0) {
                        printf("step: %d, x: %f, y: %f, vx: %f, vy: %f, vabs: %f\n", 
                                step, o.p.p.x, o.p.p.y, o.p.v.x, o.p.v.y,
                                vec3_dot(&o.p.v, &o.p.v));
                }
                if (o.steady_state)
                {
                        printf("steady state reached at step %d\n", step);
                        // Add +1 because the steady state triggered mid-step
                        // so it consumed this step
                        step++;
                        break;
                }
        }
        
        float actual_d = o.p.p.x;
        float actual_t = (float)step * w.dt;
        
        // With numerical integration and discrete steps, allow small variations
        float epsilon_d = 0.02f;
        float epsilon_t = 0.05f;
        
        if (fabsf(actual_d - expected_d) > epsilon_d)
        {
                printf("Distance mismatch: got %f, expected %f (err %f)\n", 
                       actual_d, expected_d, fabsf(actual_d - expected_d));
                ret = 1;
        }
        
        if (fabsf(actual_t - expected_t) > epsilon_t)
        {
                printf("Time mismatch: got %f, expected %f (err %f)\n", 
                       actual_t, expected_t, fabsf(actual_t - expected_t));
                ret = 1;
        }

        // Clean up
        free(m.vertices);
        free(m.indices);
        
        return ret;
}

int main(void)
{
        int fail = 0;
        fail = fail || test_sliding_friction();
        
        if (fail) {
                printf("test_friction failed\n");
        } else {
                printf("test_friction passed\n");
        }
        return fail;
}
