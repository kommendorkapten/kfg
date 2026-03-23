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

int test_friction_force_coulomb(void)
{
        int failures = 0;

        // flat ground: normal = (0,1,0)
        // velocity coming in at 45 deg: v = (3, -3, 0)
        // vn = dot((0,1,0), (3,-3,0)) = -3
        // vt = (3,-3,0) - (0,1,0)*(-3) = (3,0,0)
        // |vt| = 3
        // rc = sqrt(0.5 * 0.5) = 0.5
        // mu = sqrt(0.5 * 0.5) = 0.5
        // jn = -(1+0.5)*(-3)*1 = 4.5
        // jf = min(0.5*4.5, cr*1*3) = min(2.25, 2.1) = 2.1 (clamp ratio cr=0.7)
        // impulse = vt_dir * jf * m_inv = (1,0,0) * 2.1 * 1 = (2.1,0,0)
        {
                struct mesh m = { .restitution = 0.5f, .dynamic_mu = 0.5f };
                struct object o = {
                        .m = 1.0f, .m_inv = 1.0f,
                        .restitution = 0.5f, .dynamic_mu = 0.5f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {3, -3, 0} } }
                };
                struct vec3 expected = { .a = {2.1f, 0, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 1 (45 deg) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 2. Dead-on collision: velocity purely into surface, zero tangential
        //    v = (0, -5, 0), normal = (0,1,0)
        //    vt = (0,0,0) -> should return zero impulse
        {
                struct mesh m = { .restitution = 0.8f, .dynamic_mu = 0.6f };
                struct object o = {
                        .m = 2.0f, .m_inv = 0.5f,
                        .restitution = 0.8f, .dynamic_mu = 0.6f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {0, -5, 0} } }
                };
                struct vec3 expected = { .a = {0, 0, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 2 (dead-on) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 3. Zero friction coefficient: no friction impulse regardless of angle
        //    mu = sqrt(0 * 0.5) = 0
        {
                struct mesh m = { .restitution = 0.5f, .dynamic_mu = 0.0f };
                struct object o = {
                        .m = 1.0f, .m_inv = 1.0f,
                        .restitution = 0.5f, .dynamic_mu = 0.5f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {4, -2, 0} } }
                };
                struct vec3 expected = { .a = {0, 0, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 3 (zero mu) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 4. High friction clamps to tangential speed (clamp ratio cr=0.7)
        //    v = (0.1, -10, 0), normal = (0,1,0)
        //    vn = -10, vt = (0.1, 0, 0), |vt| = 0.1
        //    rc = sqrt(1*1) = 1, mu = sqrt(1*1) = 1
        //    jn = -(1+1)*(-10)*1 = 20
        //    jf = min(1*20, cr*1*0.1) = min(20, 0.07) = 0.07 (clamped by cr=0.7)
        //    impulse = (1,0,0) * 0.07 * 1 = (0.07,0,0)
        {
                struct mesh m = { .restitution = 1.0f, .dynamic_mu = 1.0f };
                struct object o = {
                        .m = 1.0f, .m_inv = 1.0f,
                        .restitution = 1.0f, .dynamic_mu = 1.0f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {0.1f, -10, 0} } }
                };
                struct vec3 expected = { .a = {0.07f, 0, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o, o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 4 (clamp) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 5. Diagonal tangential velocity (sliding in xz plane)
        //    v = (1, -2, 1), normal = (0,1,0)
        //    vn = -2, vt = (1,0,1), |vt| = sqrt(2)
        //    rc = sqrt(0.25*0.25) = 0.25, mu = sqrt(0.5*0.5) = 0.5
        //    jn = -(1+0.25)*(-2)*1 = 2.5
        //    jf = min(0.5*2.5, cr*1*sqrt(2)) = min(1.25, 0.9899) = 0.9899 (clamp ratio cr=0.7)
        //    vt_dir = (1/sqrt(2), 0, 1/sqrt(2))
        //    impulse = vt_dir * 0.9899 * 1.0 ≈ (0.7, 0, 0.7)
        {
                float cr = 0.7f;
                struct mesh m = { .restitution = 0.25f, .dynamic_mu = 0.5f };
                struct object o = {
                        .m = 1.0f, .m_inv = 1.0f,
                        .restitution = 0.25f, .dynamic_mu = 0.5f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {1, -2, 1} } }
                };
                float jf = cr * o.m * sqrtf(2.0f);
                float inv_sqrt2 = 1.0f / sqrtf(2.0f);
                struct vec3 expected = { .a = {jf * inv_sqrt2, 0, jf * inv_sqrt2} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 5 (diagonal) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 6. Heavier object: m=5, m_inv=0.2
        //    v = (2, -4, 0), normal = (0,1,0)
        //    vn = -4, vt = (2,0,0), |vt| = 2
        //    rc = sqrt(0.5*0.5) = 0.5, mu = sqrt(0.3*0.3) = 0.3
        //    jn = -(1+0.5)*(-4)*5 = 30
        //    jf = min(0.3*30, cr*5*2) = min(9, 7) = 7 (clamp ratio cr=0.7)
        //    impulse = (1,0,0) * 7 * 0.2 = (1.4, 0, 0)
        {
                struct mesh m = { .restitution = 0.5f, .dynamic_mu = 0.3f };
                struct object o = {
                        .m = 5.0f, .m_inv = 0.2f,
                        .restitution = 0.5f, .dynamic_mu = 0.3f,
                        .contact_normal = { .a = {0, 1, 0} },
                        .p = { .v = { .a = {2, -4, 0} } }
                };
                struct vec3 expected = { .a = {1.4f, 0, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 6 (heavy) failed: "
                               "got (%f,%f,%f)\n", result.x, result.y, result.z);
                        failures++;
                }
        }

        // 7. Inclined surface: normal = (0.707, 0.707, 0)
        //    v = (0, -2, 0)
        //    vn = dot((0.707,0.707,0), (0,-2,0)) = -1.4142
        //    vt = (0,-2,0) - (0.707,0.707,0)*(-1.4142) = (1, -1, 0) (approx)
        //    |vt| = sqrt(2) = 1.4142
        //    rc = sqrt(0.5*0.5) = 0.5, mu = sqrt(0.5*0.5) = 0.5
        //    jn = -(1+0.5)*(-1.4142)*1 = 2.1213
        //    jf = min(0.5*2.1213, cr*1*sqrt(2)) = min(1.0607, 0.9899) = 0.9899
        //         (clamp ratio cr=0.7)
        //    vt_dir = (1/sqrt(2), -1/sqrt(2), 0)
        //    impulse = vt_dir * 0.9899 * 1.0 ≈ (0.7, -0.7, 0)
        {
                float s = 0.70710678f;
                float cr = 0.7f;
                struct mesh m = { .restitution = 0.5f, .dynamic_mu = 0.5f };
                struct object o = {
                        .m = 1.0f, .m_inv = 1.0f,
                        .restitution = 0.5f, .dynamic_mu = 0.5f,
                        .contact_normal = { .a = {s, s, 0} },
                        .p = { .v = { .a = {0, -2, 0} } }
                };
                // cr clamps: jf = cr * m * |vt| = 0.7 * 1 * sqrt(2)
                float jf_val = cr * 1.0f * sqrtf(2.0f);
                float inv_sqrt2 = 1.0f / sqrtf(2.0f);
                struct vec3 expected = { .a = {jf_val * inv_sqrt2, -jf_val * inv_sqrt2, 0} };
                struct vec3 result = friction_force_coulomb(&m,
                                                            &o,
                                                            o.contact_normal);
                if (!vec3_approx(result, expected, THR)) {
                        printf("friction_force_coulomb test 7 (inclined) failed: "
                               "got (%f,%f,%f) exp (%f,%f,%f)\n",
                               result.x, result.y, result.z,
                               expected.x, expected.y, expected.z);
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
        total_failures += test_friction_force_coulomb();

        if (total_failures == 0) {
                printf("All physics tests passed!\n");
        } else {
                printf("%d tests failed.\n", total_failures);
        }

        return total_failures;
}
