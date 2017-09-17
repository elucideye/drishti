/*! -*-c++-*-
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
