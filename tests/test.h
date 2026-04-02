#ifndef TEST_H
#define TEST_H

#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "km_math.h"

struct test_entry {
        const char *name;
        // fn returns 0 on success. Positive number indicates failing
        // subtests.
        int (*fn)(void);
};

__attribute__((unused))
static int vec3_approx(struct vec3 a, struct vec3 b, float thr)
{
        return fabsf(a.x - b.x) < thr &&
               fabsf(a.y - b.y) < thr &&
               fabsf(a.z - b.z) < thr;
}

__attribute__((unused))
static int float_approx(float a, float b, float thr)
{
        return fabsf(a - b) < thr;
}

#define RUN_TESTS(entries)                                        \
int main(void)                                                    \
{                                                                 \
        int fail = 0;                                             \
        int n = (int)(sizeof(entries) / sizeof(entries[0]));      \
        for (int i = 0; i < n; i++) {                             \
                int r = entries[i].fn();                          \
                printf("  %s %s\n",                               \
                       r ? "FAIL" : " ok ",                       \
                       entries[i].name);                          \
                if (r) fail++;                                    \
        }                                                         \
        printf("\n%d/%d passed\n", n - fail, n);                  \
        return fail ? 1 : 0;                                      \
}

#endif /* TEST_H */
