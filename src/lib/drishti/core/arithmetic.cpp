/*! -*-c++-*-
  @file   arithmetic.cpp
  @author David Hirvonen
  @brief  Implementation of optimized vector arithmetic.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/arithmetic.h"
#include "drishti/core/drishti_math.h"

// clang-format off
#if defined(__arm__) || defined(__arm64__)
#  include <arm_neon.h>
#  define DO_ARM_NEON 1
#endif
// clang-format on

DRISHTI_CORE_NAMESPACE_BEGIN

template <>
float round(float x)
{
    return ::round(x);
}
template <>
double round(double x)
{
    return ::round(x);
}

// ################# ADD 32F ######################
void add32f_c(const float* pa, const float* pb, float* pc, int n)
{
    for (int i = 0; i < n; ++i)
    {
        pc[i] = pa[i] + pb[i];
    }
}

#if DO_ARM_NEON
void add32f_neon(const float* pa, const float* pb, float* pc, int n)
{
    int i = 0;
    for (; i <= (n - 4); i += 4, pa += 4, pb += 4, pc += 4)
    {
        vst1q_f32(pc, vaddq_f32(vld1q_f32(pa), vld1q_f32(pb)));
    }
    for (; i < n; i++, pa++, pb++, pc++)
    {
        pc[0] = pa[0] + pb[0];
    }
}
#endif

void add32f(const float* pa, const float* pb, float* pc, int n)
{
#if DO_ARM_NEON
    add32f_neon(pa, pb, pc, n);
#else
    add32f_c(pa, pb, pc, n);
#endif
}

// ################# ADD 16S AND 32S ######################

void add16sAnd32s_c(const int32_t* pa, const int16_t* pb, int32_t* pc, int n)
{
    for (int i = 0; i < n; i++)
    {
        pc[i] = pa[i] + pb[i];
    }
}

#if DO_ARM_NEON
void add16sAnd32s_neon(const int32_t* pa, const int16_t* pb, int32_t* pc, int n)
{
    int i = 0;
    for (; i <= (n - 4); i += 4, pa += 4, pb += 4, pc += 4)
    {
        vst1q_s32(pc, vaddq_s32(vld1q_s32(pa), vmovl_s16(vld1_s16(pb))));
    }
    for (; i < n; i++, pa++, pb++, pc++)
    {
        pc[0] = pa[0] + pb[0];
    }
}
#endif

void add16sAnd32s(const int32_t* pa, const int16_t* pb, int32_t* pc, int n)
{
#if DO_ARM_NEON
    add16sAnd32s_neon(pa, pb, pc, n);
#else
    add16sAnd32s_c(pa, pb, pc, n);
#endif
}

// ################# ADD 32S AND 32S ######################

void add16sAnd16s_c(const int16_t* pa, const int16_t* pb, int16_t* pc, int n)
{
    for (int i = 0; i < n; i++)
    {
        pc[i] = pa[i] + pb[i];
    }
}

#if DO_ARM_NEON
void add16sAnd16s_neon(const int16_t* pa, const int16_t* pb, int16_t* pc, int n)
{
    int i = 0;
    for (; i <= (n - 8); i += 8, pa += 8, pb += 8, pc += 8)
    {
        vst1q_s16(pc, vaddq_s16(vld1q_s16(pa), vld1q_s16(pb)));
    }
    for (; i <= (n - 4); i += 4, pa += 4, pb += 4, pc += 4)
    {
        vst1_s16(pc, vadd_s16(vld1_s16(pa), vld1_s16(pb)));
    }
    for (; i < n; i++, pa++, pb++, pc++)
    {
        pc[0] = pa[0] + pb[0];
    }
}
#endif

void add16sAnd16s(const int16_t* pa, const int16_t* pb, int16_t* pc, int n)
{
#if DO_ARM_NEON
    add16sAnd16s_neon(pa, pb, pc, n);
#else
    add16sAnd16s_c(pa, pb, pc, n);
#endif
}

// See: http://stackoverflow.com/questions/17998257/arm-neon-assembly-and-floating-point-rounding
// float32x4_t tmp1_ = { -0.75, -0.25, 0.25, 0.75 };
// int16x4_t tmp1 = vqmovn_s32( vcvtq_s32_f32(vaddq_f32(tmp1_, vbslq_f32(vcgtq_f32(tmp1_, zero), phalf, nhalf ))));

#if DO_ARM_NEON
static const float32x4_t v32x4f_zero = { 0.f, 0.f, 0.f, 0.f };
static const float32x4_t v32x4f_pos_half = { +0.5f, +0.5f, +0.5f, +0.5f };
static const float32x4_t v32x4f_neg_half = { -0.5f, -0.5f, -0.5f, -0.5f };
#endif

void convertFixedPoint(const float* pa, int16_t* pb, int n, int fraction)
{
    float scale = float(1 << fraction);

    int i = 0;

#if DO_ARM_NEON
    float32x4_t step = vdupq_n_f32(scale);
    for (; i <= (n - 8); i += 8, pa += 8, pb += 8)
    {
        float32x4_t lowerf = vmulq_f32(vld1q_f32(&pa[0]), step);
        float32x4_t upperf = vmulq_f32(vld1q_f32(&pa[4]), step);
        int16x4_t lower = vqmovn_s32(vcvtq_s32_f32(vaddq_f32(lowerf, vbslq_f32(vcgtq_f32(lowerf, v32x4f_zero), v32x4f_pos_half, v32x4f_neg_half))));
        int16x4_t upper = vqmovn_s32(vcvtq_s32_f32(vaddq_f32(upperf, vbslq_f32(vcgtq_f32(upperf, v32x4f_zero), v32x4f_pos_half, v32x4f_neg_half))));
        vst1q_s16(pb, vcombine_s16(lower, upper));
    }
#endif
    // process % 8 pixels from SIMD branch or all pixels in case of C implementation
    for (; i < n; i++, pa++, pb++)
    {
        pb[0] = int16_t(pa[0] * scale);
    }
}

void add16sAnd32s()
{
}

DRISHTI_CORE_NAMESPACE_END
