#include <stdio.h>
#include <math.h>

#include "km_phys.h"
#include "km_math.h"
#include "km_geom.h"

#define THR 1e-4f

static int vec3_approx(struct vec3 a, struct vec3 b, float thr)
{
        return fabsf(a.x - b.x) < thr &&
               fabsf(a.y - b.y) < thr &&
               fabsf(a.z - b.z) < thr;
}

static int float_approx(float a, float b, float thr)
{
        return fabsf(a - b) < thr;
}

int test_drag_force(void)
{
        int failures = 0;
        
        struct test_case {
                struct world w;
                struct object o;
                struct vec3 expected;
        } tests[] = {
                // 1. Zero velocity -> no drag
                {
                        .w = { .air_density = 1.225f },
                        .o = { .p = { .v = { .a = {0, 0, 0} } }, .area = 1.0f, .drag_c = 1.0f },
                        .expected = { .a = {0, 0, 0} }
                },
                // 2. Unit velocity along X
                {
                        .w = { .air_density = 1.225f },
                        .o = { .p = { .v = { .a = {1, 0, 0} } }, .area = 1.0f, .drag_c = 1.0f },
                        .expected = { .a = {0.6125f, 0, 0} } // 0.5 * 1.225 * 1 * 1 * 1^2
                },
                // 3. Different air density
                {
                        .w = { .air_density = 2.0f },
                        .o = { .p = { .v = { .a = {0, -2, 0} } }, .area = 1.0f, .drag_c = 1.0f },
                        .expected = { .a = {0, -4.0f, 0} } // 0.5 * 2.0 * 1 * 1 * 2^2, pointing -y
                },
                // 4. Different drag coefficient
                {
                        .w = { .air_density = 1.0f },
                        .o = { .p = { .v = { .a = {0, 0, 3} } }, .area = 1.0f, .drag_c = 0.5f },
                        .expected = { .a = {0, 0, 2.25f} } // 0.5 * 1.0 * 1.0 * 0.5 * 3^2
                },
                // 5. Zero area
                {
                        .w = { .air_density = 1.2f },
                        .o = { .p = { .v = { .a = {10, 0, 0} } }, .area = 0.0f, .drag_c = 1.0f },
                        .expected = { .a = {0, 0, 0} }
                },
                // 6. Zero air density
                {
                        .w = { .air_density = 0.0f },
                        .o = { .p = { .v = { .a = {10, 0, 0} } }, .area = 2.0f, .drag_c = 1.0f },
                        .expected = { .a = {0, 0, 0} }
                },
                // 7. Diagonal velocity vector
                {
                        .w = { .air_density = 2.0f },
                        .o = { .p = { .v = { .a = {1, 1, 0} } }, .area = 1.0f, .drag_c = 1.0f }, // ||v||^2 = 2, dir = (1/sqrt(2), 1/sqrt(2), 0)
                        .expected = { .a = {1.41421356f, 1.41421356f, 0} } // 0.5 * 2.0 * 1.0 * 1.0 * 2.0 * (1/sqrt(2)) = sqrt(2)
                },
                // 8. Negative velocity coordinates
                {
                        .w = { .air_density = 1.0f },
                        .o = { .p = { .v = { .a = {-2, 0, 0} } }, .area = 2.0f, .drag_c = 0.5f },
                        .expected = { .a = {-2.0f, 0, 0} } // 0.5 * 1.0 * 2.0 * 0.5 * 4 * (-1)
                }
        };

        int n_tests = sizeof(tests) / sizeof(tests[0]);
        for (int i = 0; i < n_tests; i++) {
                struct vec3 result = drag_force(&tests[i].w, &tests[i].o);
                if (!vec3_approx(result, tests[i].expected, THR)) {
                        printf("drag_force test %d failed\n", i);
                        failures++;
                }
        }
        return failures;
}

int test_friction_force_dyn(void)
{
        int failures = 0;

        struct test_case {
                struct mesh m;
                struct object o;
                struct vec3 expected;
        } tests[] = {
                // 1. Standard horizontal sliding
                {
                        .m = { .dynamic_mu = 0.5f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} }, .p = { .v = { .a = {2, 0, 0} } } },
                        .expected = { .a = {-0.5f * 1.0f * KM_PHYS_G, 0, 0} } 
                },
                // 2. Zero velocity -> no dynamic friction or directional issue (wait, zero velocity might yield 0, check implementation)
                {
                        .m = { .dynamic_mu = 0.5f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} }, .p = { .v = { .a = {0, 0, 0} } } },
                        .expected = { .a = {0, 0, 0} } // TV is zero, so usually we'd expect 0 vector 
                },
                // 3. Different mass
                {
                        .m = { .dynamic_mu = 0.2f },
                        .o = { .m = 10.0f, .dynamic_mu = 0.2f, .contact_normal = { .a = {0, 1, 0} }, .p = { .v = { .a = {-1, 0, 0} } } },
                        .expected = { .a = {0.2f * 10.0f * KM_PHYS_G, 0, 0} } 
                },
                // 4. Velocity parallel to normal (pure bouncing, orthogonal tangent velocity should be 0)
                {
                        .m = { .dynamic_mu = 0.5f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} }, .p = { .v = { .a = {0, 2, 0} } } },
                        .expected = { .a = {0, 0, 0} }
                },
                // 5. Inclined collision normal (e.g. 45 degrees)
                {
                        .m = { .dynamic_mu = 0.4f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.9f, .contact_normal = { .a = {0.70710678f, 0.70710678f, 0} }, .p = { .v = { .a = {1, -1, 0} } } },
                        .expected = { .a = {-0.6f * 1.0f * KM_PHYS_G * 0.70710678f * 0.70710678f, 0.6f * 1.0f * KM_PHYS_G * 0.70710678f * 0.70710678f, 0} }  
                },
                // 6. Zero friction coefficient
                {
                        .m = { .dynamic_mu = 0.0f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} }, .p = { .v = { .a = {2, 0, 0} } } },
                        .expected = { .a = {0, 0, 0} }
                },
                // 7. Normal orthogonal to up vector (wall sliding - depending on physics implementation of fN this might be 0)
                {
                        .m = { .dynamic_mu = 0.5f },
                        .o = { .m = 1.0f, .dynamic_mu = 0.5f, .contact_normal = { .a = {1, 0, 0} }, .p = { .v = { .a = {0, -2, 0} } } },
                        .expected = { .a = {0, 0, 0} } // dot(up, normal) = 0, so fN = 0 -> friction = 0
                }
        };

        int n_tests = sizeof(tests) / sizeof(tests[0]);
        for (int i = 0; i < n_tests; i++) {
                struct vec3 result = friction_force_dyn(&tests[i].m, &tests[i].o);
                if (!vec3_approx(result, tests[i].expected, THR)) {
                        printf("friction_force_dyn test %d failed\n", i);
                        failures++;
                }
        }
        return failures;
}

int test_friction_force_stat(void)
{
        int failures = 0;

        struct test_case {
                struct mesh m;
                struct object o;
                float expected;
        } tests[] = {
                // 1. Standard flat surface
                {
                        .m = { .static_mu = 0.5f },
                        .o = { .m = 1.0f, .static_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} } },
                        .expected = 0.5f * 1.0f * KM_PHYS_G 
                },
                // 2. Different mass
                {
                        .m = { .static_mu = 0.5f },
                        .o = { .m = 2.0f, .static_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} } },
                        .expected = 0.5f * 2.0f * KM_PHYS_G
                },
                // 3. Different mu
                {
                        .m = { .static_mu = 0.8f },
                        .o = { .m = 1.0f, .static_mu = 0.2f, .contact_normal = { .a = {0, 1, 0} } },
                        .expected = 0.4f * 1.0f * KM_PHYS_G 
                },
                // 4. Inclined surface 45 degrees
                {
                        .m = { .static_mu = 0.5f },
                        .o = { .m = 1.0f, .static_mu = 0.5f, .contact_normal = { .a = {0.70710678f, 0.70710678f, 0} } },
                        .expected = 0.5f * 1.0f * KM_PHYS_G * 0.70710678f 
                },
                // 5. Wall (normal perfectly horizontal)
                {
                        .m = { .static_mu = 0.5f },
                        .o = { .m = 1.0f, .static_mu = 0.5f, .contact_normal = { .a = {1, 0, 0} } },
                        .expected = 0.0f 
                },
                // 6. Zero static friction
                {
                        .m = { .static_mu = 0.0f },
                        .o = { .m = 1.0f, .static_mu = 0.5f, .contact_normal = { .a = {0, 1, 0} } },
                        .expected = 0.0f
                },
                // 7. Upside down surface (normal points down)
                {
                        .m = { .static_mu = 0.5f },
                        .o = { .m = 1.0f, .static_mu = 0.5f, .contact_normal = { .a = {0, -1, 0} } },
                        .expected = 0.5f * 1.0f * KM_PHYS_G 
                }
        };

        int n_tests = sizeof(tests) / sizeof(tests[0]);
        for (int i = 0; i < n_tests; i++) {
                float result = friction_force_stat(&tests[i].m, &tests[i].o);
                if (!float_approx(result, tests[i].expected, THR)) {
                        printf("friction_force_stat test %d failed\n", i);
                        failures++;
                }
        }
        return failures;
}

int main(void)
{
        int total_failures = 0;
        
        total_failures += test_drag_force();
        total_failures += test_friction_force_dyn();
        total_failures += test_friction_force_stat();
        
        if (total_failures == 0) {
                printf("All physics tests passed!\n");
        } else {
                printf("%d tests failed.\n", total_failures);
        }
        
        return total_failures;
}
