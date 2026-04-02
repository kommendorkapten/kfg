#include "test.h"

#define THR 1e-5f

static int test_vec3_add(void);
static int test_vec3_sub(void);
static int test_vec3_scalarm(void);
static int test_vec3_norm(void);
static int test_vec3_dot(void);
static int test_vec3_cross(void);
static int test_km_rsqrt(void);
static int test_vec3_iszero(void);

// Test vec3_add with several vector combinations.
static int test_vec3_add(void)
{
        int ret = 0;

        struct {
                struct vec3 a;
                struct vec3 b;
                struct vec3 exp;
                const char* label;
        } cases[] = {
                // zero + zero
                {
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "zero+zero",
                },
                // positive components
                {
                        { .a = {1.0f, 2.0f, 3.0f} },
                        { .a = {4.0f, 5.0f, 6.0f} },
                        { .a = {5.0f, 7.0f, 9.0f} },
                        "pos+pos",
                },
                // negative components
                {
                        { .a = {-1.0f, -2.0f, -3.0f} },
                        { .a = {-4.0f, -5.0f, -6.0f} },
                        { .a = {-5.0f, -7.0f, -9.0f} },
                        "neg+neg",
                },
                // mixed signs
                {
                        { .a = {1.0f, -2.0f, 3.0f} },
                        { .a = {-1.0f, 2.0f, -3.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "cancel",
                },
                // one side zero
                {
                        { .a = {3.0f, -7.0f, 0.5f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {3.0f, -7.0f, 0.5f} },
                        "v+zero",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 r = vec3_add(cases[i].a,
                                         cases[i].b);
                if (!vec3_approx(r, cases[i].exp, THR))
                {
                        printf("add %s: "
                               "got (%.6f,%.6f,%.6f) "
                               "exp (%.6f,%.6f,%.6f)\n",
                               cases[i].label,
                               r.x, r.y, r.z,
                               cases[i].exp.x,
                               cases[i].exp.y,
                               cases[i].exp.z);
                        ret = 1;
                }
        }

        return ret;
}

// Test vec3_sub with several vector combinations.
static int test_vec3_sub(void)
{
        int ret = 0;

        struct {
                struct vec3 a;
                struct vec3 b;
                struct vec3 exp;
                const char* label;
        } cases[] = {
                // zero - zero
                {
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "zero-zero",
                },
                // a - b positive result
                {
                        { .a = {5.0f, 7.0f, 9.0f} },
                        { .a = {1.0f, 2.0f, 3.0f} },
                        { .a = {4.0f, 5.0f, 6.0f} },
                        "pos-pos",
                },
                // a - a = zero
                {
                        { .a = {3.0f, -4.0f, 5.0f} },
                        { .a = {3.0f, -4.0f, 5.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "self",
                },
                // negative result
                {
                        { .a = {1.0f, 1.0f, 1.0f} },
                        { .a = {2.0f, 3.0f, 4.0f} },
                        { .a = {-1.0f, -2.0f, -3.0f} },
                        "neg result",
                },
                // mixed signs
                {
                        { .a = {-1.0f, 2.0f, -3.0f} },
                        { .a = {4.0f, -5.0f, 6.0f} },
                        { .a = {-5.0f, 7.0f, -9.0f} },
                        "mixed",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 r = vec3_sub(cases[i].a,
                                         cases[i].b);
                if (!vec3_approx(r, cases[i].exp, THR))
                {
                        printf("sub %s: "
                               "got (%.6f,%.6f,%.6f) "
                               "exp (%.6f,%.6f,%.6f)\n",
                               cases[i].label,
                               r.x, r.y, r.z,
                               cases[i].exp.x,
                               cases[i].exp.y,
                               cases[i].exp.z);
                        ret = 1;
                }
        }

        return ret;
}

// Test vec3_scalarm with positive, negative and
// zero scalars.
static int test_vec3_scalarm(void)
{
        int ret = 0;

        struct {
                struct vec3 v;
                float s;
                struct vec3 exp;
                const char* label;
        } cases[] = {
                // identity
                {
                        { .a = {1.0f, 2.0f, 3.0f} },
                        1.0f,
                        { .a = {1.0f, 2.0f, 3.0f} },
                        "identity",
                },
                // zero scalar
                {
                        { .a = {1.0f, 2.0f, 3.0f} },
                        0.0f,
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "zero",
                },
                // positive scalar
                {
                        { .a = {1.0f, -2.0f, 3.0f} },
                        2.0f,
                        { .a = {2.0f, -4.0f, 6.0f} },
                        "x2",
                },
                // negative scalar
                {
                        { .a = {1.0f, -2.0f, 3.0f} },
                        -3.0f,
                        { .a = {-3.0f, 6.0f, -9.0f} },
                        "x-3",
                },
                // fractional scalar
                {
                        { .a = {4.0f, 6.0f, -8.0f} },
                        0.5f,
                        { .a = {2.0f, 3.0f, -4.0f} },
                        "x0.5",
                },
                // zero vector
                {
                        { .a = {0.0f, 0.0f, 0.0f} },
                        5.0f,
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "zero vec",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 r = vec3_scalarm(cases[i].v,
                                             cases[i].s);
                if (!vec3_approx(r, cases[i].exp, THR))
                {
                        printf("scalarm %s: "
                               "got (%.6f,%.6f,%.6f) "
                               "exp (%.6f,%.6f,%.6f)\n",
                               cases[i].label,
                               r.x, r.y, r.z,
                               cases[i].exp.x,
                               cases[i].exp.y,
                               cases[i].exp.z);
                        ret = 1;
                }
        }

        return ret;
}

// Test vec3_norm produces a unit-length vector
// in the same direction.
static int test_vec3_norm(void)
{
        int ret = 0;

        struct {
                struct vec3 v;
                const char* label;
        } cases[] = {
                {{ .a = {1.0f, 0.0f, 0.0f} }, "+x"},
                {{ .a = {0.0f, 1.0f, 0.0f} }, "+y"},
                {{ .a = {0.0f, 0.0f, 1.0f} }, "+z"},
                {{ .a = {-1.0f, 0.0f, 0.0f} }, "-x"},
                {{ .a = {0.0f, -5.0f, 0.0f} }, "-5y"},
                {{ .a = {3.0f, 4.0f, 0.0f} }, "3,4,0"},
                {{ .a = {1.0f, 1.0f, 1.0f} }, "1,1,1"},
                {{ .a = {-2.0f, 3.0f, -6.0f} }, "-2,3,-6"},
                {{ .a = {0.001f, 0.0f, 0.0f} }, "small"},
                {{ .a = {100.0f, 200.0f, 300.0f} }, "large"},
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 r = vec3_norm(cases[i].v);

                // length must be 1
                float len = sqrtf(vec3_dot(r, r));
                if (fabsf(len - 1.0f) > THR)
                {
                        printf("norm %s: "
                               "len=%.6f (exp 1.0)\n",
                               cases[i].label, len);
                        ret = 1;
                }

                // must point in same direction (dot > 0)
                float d = vec3_dot(r, cases[i].v);
                if (d < 0.0f)
                {
                        printf("norm %s: "
                               "flipped direction\n",
                               cases[i].label);
                        ret = 1;
                }
        }

        return ret;
}

// Test vec3_dot for orthogonal, parallel and
// anti-parallel vectors.
static int test_vec3_dot(void)
{
        int ret = 0;

        struct {
                struct vec3 a;
                struct vec3 b;
                float exp;
                const char* label;
        } cases[] = {
                // orthogonal = 0
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 1.0f, 0.0f} },
                        0.0f,
                        "ortho xy",
                },
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 1.0f} },
                        0.0f,
                        "ortho xz",
                },
                {
                        { .a = {0.0f, 1.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 1.0f} },
                        0.0f,
                        "ortho yz",
                },
                // parallel
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {3.0f, 0.0f, 0.0f} },
                        3.0f,
                        "parallel x",
                },
                // anti-parallel
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {-2.0f, 0.0f, 0.0f} },
                        -2.0f,
                        "anti x",
                },
                // general case: 1*4 + 2*5 + 3*6 = 32
                {
                        { .a = {1.0f, 2.0f, 3.0f} },
                        { .a = {4.0f, 5.0f, 6.0f} },
                        32.0f,
                        "general",
                },
                // negative components
                {
                        { .a = {-1.0f, -2.0f, -3.0f} },
                        { .a = {-4.0f, -5.0f, -6.0f} },
                        32.0f,
                        "neg neg",
                },
                // self dot = |v|^2
                {
                        { .a = {3.0f, 4.0f, 0.0f} },
                        { .a = {3.0f, 4.0f, 0.0f} },
                        25.0f,
                        "self",
                },
                // zero vector
                {
                        { .a = {0.0f, 0.0f, 0.0f} },
                        { .a = {1.0f, 2.0f, 3.0f} },
                        0.0f,
                        "zero",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                float r = vec3_dot(cases[i].a,
                                   cases[i].b);
                if (fabsf(r - cases[i].exp) > THR)
                {
                        printf("dot %s: "
                               "got %.6f exp %.6f\n",
                               cases[i].label,
                               r, cases[i].exp);
                        ret = 1;
                }
        }

        return ret;
}

/*
 * Test vec3_cross against known results and
 * properties: anti-commutativity, orthogonality
 * to both inputs, and right-hand rule.
 */
static int test_vec3_cross(void)
{
        int ret = 0;

        struct {
                struct vec3 a;
                struct vec3 b;
                struct vec3 exp;
                const char* label;
        } cases[] = {
                // basis vectors: x cross y = z
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 1.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 1.0f} },
                        "x*y=z",
                },
                // y cross z = x
                {
                        { .a = {0.0f, 1.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 1.0f} },
                        { .a = {1.0f, 0.0f, 0.0f} },
                        "y*z=x",
                },
                // z cross x = y
                {
                        { .a = {0.0f, 0.0f, 1.0f} },
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 1.0f, 0.0f} },
                        "z*x=y",
                },
                // anti-commutative: y cross x = -z
                {
                        { .a = {0.0f, 1.0f, 0.0f} },
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, -1.0f} },
                        "y*x=-z",
                },
                // parallel => zero
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        { .a = {3.0f, 0.0f, 0.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "parallel",
                },
                // self cross => zero
                {
                        { .a = {2.0f, 3.0f, 4.0f} },
                        { .a = {2.0f, 3.0f, 4.0f} },
                        { .a = {0.0f, 0.0f, 0.0f} },
                        "self",
                },
                // general: (1,2,3) x (4,5,6)
                // = (2*6-3*5, 3*4-1*6, 1*5-2*4)
                // = (-3, 6, -3)
                {
                        { .a = {1.0f, 2.0f, 3.0f} },
                        { .a = {4.0f, 5.0f, 6.0f} },
                        { .a = {-3.0f, 6.0f, -3.0f} },
                        "general",
                },
                // negative components
                {
                        { .a = {-1.0f, 2.0f, -3.0f} },
                        { .a = {4.0f, -5.0f, 6.0f} },
                        // (2*6-(-3)*(-5), (-3)*4-(-1)*6,
                        //  (-1)*(-5)-2*4)
                        // = (12-15, -12+6, 5-8) = (-3,-6,-3)
                        { .a = {-3.0f, -6.0f, -3.0f} },
                        "neg mix",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                struct vec3 r = vec3_cross(cases[i].a,
                                           cases[i].b);
                if (!vec3_approx(r, cases[i].exp, THR))
                {
                        printf("cross %s: "
                               "got (%.6f,%.6f,%.6f) "
                               "exp (%.6f,%.6f,%.6f)\n",
                               cases[i].label,
                               r.x, r.y, r.z,
                               cases[i].exp.x,
                               cases[i].exp.y,
                               cases[i].exp.z);
                        ret = 1;
                }

                // result must be orthogonal to both inputs
                float da = vec3_dot(r, cases[i].a);
                float db = vec3_dot(r, cases[i].b);
                if (fabsf(da) > THR || fabsf(db) > THR)
                {
                        printf("cross %s: "
                               "not orthogonal "
                               "(da=%.6f db=%.6f)\n",
                               cases[i].label, da, db);
                        ret = 1;
                }
        }

        return ret;
}

// Test km_rsqrt against 1/sqrt(x) for several
// values. The fast inverse sqrt has limited
// precision so use a looser threshold.
static int test_km_rsqrt(void)
{
        int ret = 0;
        // km_rsqrt does one NR step, ~1% accuracy
        float thr = 0.02f;

        float vals[] = {
                0.25f, 0.5f, 1.0f, 2.0f, 4.0f,
                9.0f, 16.0f, 100.0f, 0.01f, 10000.0f,
        };
        int n = (int)(sizeof(vals) / sizeof(vals[0]));

        for (int i = 0; i < n; i++)
        {
                float r = km_rsqrt(vals[i]);
                float exp = 1.0f / sqrtf(vals[i]);
                float rel = fabsf(r - exp) / exp;

                if (rel > thr)
                {
                        printf("rsqrt(%.4f): "
                               "got %.6f exp %.6f "
                               "rel err %.6f\n",
                               vals[i], r, exp, rel);
                        ret = 1;
                }
        }

        return ret;
}

// Test vec3_iszero for zero, near-zero and
// non-zero vectors.
static int test_vec3_iszero(void)
{
        int ret = 0;

        struct {
                struct vec3 v;
                int exp;
                const char* label;
        } cases[] = {
                // exact zero
                {
                        { .a = {0.0f, 0.0f, 0.0f} },
                        1,
                        "zero",
                },
                // very small (below 1e-8 squared len)
                {
                        { .a = {1e-5f, 0.0f, 0.0f} },
                        1,
                        "tiny x",
                },
                {
                        { .a = {0.0f, 1e-5f, 1e-5f} },
                        1,
                        "tiny yz",
                },
                // not zero
                {
                        { .a = {1.0f, 0.0f, 0.0f} },
                        0,
                        "unit x",
                },
                {
                        { .a = {0.0f, -1.0f, 0.0f} },
                        0,
                        "-unit y",
                },
                {
                        { .a = {0.01f, 0.0f, 0.0f} },
                        0,
                        "small x",
                },
                // all negative, not zero
                {
                        { .a = {-1.0f, -1.0f, -1.0f} },
                        0,
                        "neg all",
                },
        };
        int n = (int)(sizeof(cases) / sizeof(cases[0]));

        for (int i = 0; i < n; i++)
        {
                int r = vec3_iszero(cases[i].v);
                if (r != cases[i].exp)
                {
                        printf("iszero %s: "
                               "got %d exp %d\n",
                               cases[i].label,
                               r, cases[i].exp);
                        ret = 1;
                }
        }

        return ret;
}

static struct test_entry tests[] = {
        {"vec3_add",     test_vec3_add},
        {"vec3_sub",     test_vec3_sub},
        {"vec3_scalarm", test_vec3_scalarm},
        {"vec3_norm",    test_vec3_norm},
        {"vec3_dot",     test_vec3_dot},
        {"vec3_cross",   test_vec3_cross},
        {"km_rsqrt",     test_km_rsqrt},
        {"vec3_iszero",  test_vec3_iszero},
};
RUN_TESTS(tests)
