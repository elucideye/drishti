/*!
  @file   test-drishti.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

int gauze_main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include "drishti/core/hungarian.h"
#include <vector>

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

    cv::Mat1f C(points1.size(), points2.size(), 0.f);

    for (int i = 0; i < points1.size(); i++)
    {
        for (int j = 0; j < points2.size(); j++)
        {
            C(i,j) = cv::norm(points1[i] - points2[j]);
        }
    }

    drishti::core::DMatchVec outMatches;
    drishti::core::IntVec inliers1, inliers2;
    drishti::core::hungarian(C, outMatches, inliers1, inliers2, C.rows, C.cols);

    ASSERT_EQ(outMatches.size(), 3);
    for(int i = 0; i < 3; i++)
    {
        ASSERT_EQ(outMatches[i].queryIdx, outMatches[i].trainIdx);
    }
}
