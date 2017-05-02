/*!
  @file   test-convert.cpp
  @author David Hirvonen
  @brief  Google fixture with various subtests for optimized conversions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/core/convert.h"

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

BEGIN_EMPTY_NAMESPACE

/*
 * Basic class construction
 */

static const int rgba[] = { 2, 1, 0, 3 };

static std::vector<cv::Mat> unpack_test(const cv::Size& size)
{
    cv::Mat4b src(size, cv::Vec4b(0, 1, 2, 3));

    std::vector<cv::Mat> dst{
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size())
    };

    std::vector<drishti::core::PlaneInfo> table{ { dst[0], rgba[0] }, { dst[1], rgba[1] }, { dst[2], rgba[2] }, { dst[3], rgba[3] } };

    drishti::core::unpack(src, table);

    return dst;
}

TEST(ChannelConversion, unpack_mul_16)
{
    auto dst = unpack_test({ 100, 160 });
    for (int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero(dst[i] == rgba[i]);
        ASSERT_EQ(count, dst[i].total());
    }
}

TEST(ChannelConversion, unpack_rem_16)
{
    auto dst = unpack_test({ 100, 161 });
    for (int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero(dst[i] == rgba[i]);
        ASSERT_EQ(count, dst[i].total());
    }
}

static std::vector<cv::Mat> convert_test(const cv::Size& size)
{
    cv::Mat4b src(size, cv::Vec4b(0, 1, 2, 3));

    std::vector<cv::Mat> dst{
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size())
    };

    std::vector<drishti::core::PlaneInfo> table{ { dst[0], rgba[0] }, { dst[1], rgba[1] }, { dst[2], rgba[2] }, { dst[3], rgba[3] } };

    drishti::core::convertU8ToF32(src, table);

    return dst;
}

TEST(ChannelConversion, convert_mul_16)
{
    auto dst = convert_test({ 100, 160 });

    for (int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero(dst[i] == float(rgba[i]));
        ASSERT_EQ(count, dst[i].total());
    }
}

TEST(ChannelConversion, convert_rem_16)
{
    auto dst = convert_test({ 100, 161 });

    for (int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero(dst[i] == float(rgba[i]));
        ASSERT_EQ(count, dst[i].total());
    }
}

END_EMPTY_NAMESPACE
