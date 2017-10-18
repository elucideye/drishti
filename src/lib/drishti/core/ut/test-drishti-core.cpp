/*! -*-c++-*-
  @file   test-drishti-core.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/core/convert.h"
#include "drishti/core/hungarian.h"
#include <vector>

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

BEGIN_EMPTY_NAMESPACE

TEST(HungarianAssignment, hungarian)
{
    std::vector<cv::Point2f> points1
    {
        { 1.f, 1.f },
        { 2.f, 2.f },
        { 3.f, 3.f }
    };

    std::vector<cv::Point2f> points2
    {
        { 0.0f, 1.0f },
        { 2.5f, 1.5f },
        { 3.0f, 2.5f }
    };
    
    std::unordered_map<int, int> direct_assignment;
    std::unordered_map<int, int> reverse_assignment;
    std::vector<std::vector<double>> C(points1.size(), std::vector<double>(points2.size(), std::numeric_limits<float>::max()));

    for (int i = 0; i < points1.size(); i++)
    {
        for (int j = 0; j < points2.size(); j++)
        {
            C[i][j] = cv::norm(points1[i] - points2[j]);
        }
    }
    
    drishti::core::MinimizeLinearAssignment(C, direct_assignment, reverse_assignment);
    
    ASSERT_EQ(direct_assignment.size(), 3);
    for(const auto &m : direct_assignment)
    {
        ASSERT_EQ(m.first, m.second);
    }
}

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

