/*! -*-c++-*-
  @file   convert.cpp
  @author David Hirvonen
  @brief  Implementation of optimized unpack and format conversion routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/convert.h"

// clang-format off
#if defined (__arm__) || defined(__arm64__)
#  include <arm_neon.h>
#  define USE_SIMD 1
#elif __APPLE__
#  include "TargetConditionals.h"
#  if !TARGET_IPHONE_SIMULATOR
#    include "NEON_2_SSE.h"
#    define USE_SIMD 1
#  else
#    define USE_SIMD 0
#  endif
#else
#  if (defined(__SSE2__) || defined(__x86_64__) || defined(__AVX2__)) && BUILD_REGRESSION_SIMD
#    define USE_SIMD 1
#    include "NEON_2_SSE.h"
#  else
#    define USE_SIMD 0
#  endif
#endif
// clang-format on

DRISHTI_CORE_NAMESPACE_BEGIN

#if USE_SIMD

void convertU8ToF32(const cv::Mat4b& input, std::vector<PlaneInfo>& planes)
{
    CV_Assert(!input.empty());
    CV_Assert(planes.size() > 0);

    const int stepA = 16;
    std::vector<float*> ptrs(planes.size());
    std::vector<float32x4_t> alphas(planes.size());
    for (int i = 0; i < planes.size(); i++)
    {
        alphas[i] = vdupq_n_f32(planes[i].alpha);
    }

    for (int x = 0, y = 0; y < input.rows; y++)
    {
        const cv::Vec4b* ptrA = input.ptr<cv::Vec4b>(y);
        for (int i = 0; i < planes.size(); i++)
        {
            ptrs[i] = planes[i].plane.ptr<float>(y);
        }

        for (x = 0; x <= input.cols - stepA; x += stepA, ptrA += stepA)
        {
            uint8x16x4_t a = vld4q_u8(reinterpret_cast<const uint8_t*>(ptrA));
            for (int i = 0; i < planes.size(); ptrs[i] += stepA, i++)
            {
                const auto& c = planes[i].channel;

                uint8x8_t l, u;
                l = vget_low_u8(a.val[c]);
                u = vget_high_u8(a.val[c]);
                uint16x8_t b1 = vmovl_u8(l);
                uint16x8_t b2 = vmovl_u8(u);

                uint16x4_t l1, u1;
                l1 = vget_low_u16(b1);
                u1 = vget_high_u16(b1);
                float32x4_t c1a = vcvtq_f32_u32(vmovl_u16(l1));
                float32x4_t c1b = vcvtq_f32_u32(vmovl_u16(u1));

                l1 = vget_low_u16(b2);
                u1 = vget_high_u16(b2);
                float32x4_t c2a = vcvtq_f32_u32(vmovl_u16(l1));
                float32x4_t c2b = vcvtq_f32_u32(vmovl_u16(u1));

                vst1q_f32(ptrs[i] + 0, vmulq_f32(c1a, alphas[i]));
                vst1q_f32(ptrs[i] + 4, vmulq_f32(c1b, alphas[i]));
                vst1q_f32(ptrs[i] + 8, vmulq_f32(c2a, alphas[i]));
                vst1q_f32(ptrs[i] + 12, vmulq_f32(c2b, alphas[i]));
            }
        }

        // for now, process one at a time
        for (; x < input.cols; x++, ptrA++)
        {
            const cv::Vec4b& pix = ptrA[0];
            for (int i = 0; i < planes.size(); ptrs[i]++, i++)
            {
                ptrs[i][0] = static_cast<float>(pix[planes[i].channel]) * alphas[i][0];
            }
        }
    }
}

void unpack(const cv::Mat4b& input, std::vector<PlaneInfo>& planes)
{
    std::vector<uint8_t*> ptrs(planes.size());

    for (int x, y = 0; y < input.rows; y++)
    {
        // Initialize output row pointers:
        for (int i = 0; i < planes.size(); i++)
        {
            ptrs[i] = planes[i].plane.ptr<uint8_t>(y);
        }

        const int stepA = 16;
        const cv::Vec4b* ptrA = input.ptr<cv::Vec4b>(y);
        for (x = 0; x <= input.cols - stepA; x += stepA, ptrA += stepA)
        {
            uint8x16x4_t a = vld4q_u8(reinterpret_cast<const uint8_t*>(ptrA));
            for (int i = 0; i < planes.size(); ptrs[i] += stepA, i++)
            {
                const auto& c = planes[i].channel;
                vst1q_u8(ptrs[i], a.val[c]);
            }
        }

        // for now, process one at a time
        for (; x < input.cols; x++, ptrA++)
        {
            const cv::Vec4b& pix = ptrA[0];
            for (int i = 0; i < planes.size(); ptrs[i]++, i++)
            {
                ptrs[i][0] = pix[planes[i].channel];
            }
        }
    }
}

#else

void convertU8ToF32(const cv::Mat4b& input, std::vector<PlaneInfo>& planes)
{
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    for (auto& p : planes)
    {
        channels[p.channel].convertTo(p.plane, CV_32F, p.alpha);
    }
}

void unpack(const cv::Mat4b& input, std::vector<PlaneInfo>& planes)
{
    std::vector<cv::Mat> channels;
    cv::split(input, channels);
    for (auto& p : planes)
    {
        channels[p.channel].copyTo(p.plane);
    }
}

#endif

DRISHTI_CORE_NAMESPACE_END
