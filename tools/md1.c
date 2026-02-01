#include <stdio.h>
#include <arm_neon.h>

#if defined(__GNUC__) || defined(__clang__)
    #define ckd_add(r, a, b) __builtin_add_overflow(a, b, r)
    #define ckd_sub(r, a, b) __builtin_sub_overflow(a, b, r)
    #define ckd_mul(r, a, b) __builtin_mul_overflow(a, b, r)

#elif defined(_MSC_VER)
    #include <intsafe.h>
    // MSVC uses different API
    #define ckd_add(r, a, b) (IntAdd(a, b, r) != S_OK)

#else
    #warning "unrecognized compiler"
#endif

void add_arrays_neon(float *a, float *b, float *result, int n);
float dot_product_neon(const float *a, const float *b, int n);

int main(int argc, char** argv)
{
        (void)argc;
        (void)argv;

        printf("hello md1\n");

        int a = 1000000000;
        int b = 2000000000;
        int result;

        if (ckd_add(&result, a, b)) {
                printf("Overflow detected!\n");
        } else {
                printf("Result: %d\n", result);
        }

        float v1[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float v2[4] = {1.0f, 0.0f, 0.0f, 0.0f};
        float dp = dot_product_neon(v1, v2, 4);

        printf("%f\n", dp);

        return 0;
}

void add_arrays_neon(float *a, float *b, float *result, int n) {
    int i = 0;

    // Process 4 floats at a time
    for (; i <= n - 4; i += 4) {
        float32x4_t va = vld1q_f32(&a[i]);   // Load 4 floats
        float32x4_t vb = vld1q_f32(&b[i]);   // Load 4 floats
        float32x4_t vr = vaddq_f32(va, vb);  // Add them
        vst1q_f32(&result[i], vr);           // Store result
    }

    // Handle remainder
    for (; i < n; i++) {
        result[i] = a[i] + b[i];
    }
}

float dot_product_neon(const float *a, const float *b, int n) {
    float32x4_t sum = vdupq_n_f32(0.0f);
    int i = 0;

    for (; i <= n - 4; i += 4) {
        float32x4_t va = vld1q_f32(&a[i]);
        float32x4_t vb = vld1q_f32(&b[i]);
        sum = vfmaq_f32(sum, va, vb);  // sum += va * vb
    }

    // Horizontal sum of 4 lanes
    float result = vaddvq_f32(sum);

    // Remainder
    for (; i < n; i++) {
        result += a[i] * b[i];
    }

    return result;
}
