/*!
  @file   test-convert.cpp
  @author David Hirvonen
  @brief  Google fixture with various subtests for optimized conversions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/core/convert.h"

#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }

BEGIN_EMPTY_NAMESPACE

/*
 * Basic class construction
 */

TEST(EyeSegmenter, unpack)
{
    cv::Mat4b src(100, 100, cv::Vec4b(0,1,2,3));
    
    std::vector<cv::Mat> dst
    {
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size()),
        cv::Mat1b(src.size())
    };
    
    int rgba[] = { 2, 1, 0, 3 };
    std::vector<drishti::core::PlaneInfo> table { {dst[0],rgba[0]}, {dst[1],rgba[1]}, {dst[2],rgba[2]}, {dst[3],rgba[3]} };
    
    drishti::core::unpack(src, table);
    
    for(int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero( dst[i] == rgba[i] );
        ASSERT_EQ(count, dst[i].total());
    }
}

TEST(EyeSegmenter, convert)
{
    cv::Mat4b src(100, 100, cv::Vec4b(0,1,2,3));
    
    std::vector<cv::Mat> dst
    {
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size()),
        cv::Mat1f(src.size())
    };
    
    int rgba[] = { 2, 1, 0, 3 };
    std::vector<drishti::core::PlaneInfo> table { {dst[0],rgba[0]}, {dst[1],rgba[1]}, {dst[2],rgba[2]}, {dst[3],rgba[3]} };
    
    drishti::core::convertU8ToF32(src, table);
    
    for(int i = 0; i < 4; i++)
    {
        int count = cv::countNonZero( dst[i] == float(rgba[i]) );
        ASSERT_EQ(count, dst[i].total());
    }
}

END_EMPTY_NAMESPACE


