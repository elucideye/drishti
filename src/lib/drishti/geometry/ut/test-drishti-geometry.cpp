/*! -*-c++-*-
  @file   test-drishti-geometry.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/geometry/Ellipse.h"
#include "drishti/geometry/intersectConicLine.h"

TEST(Ellipse, EllipseLineIntersection2)
{
    const cv::RotatedRect E({ 0.f, 0.f }, { 2.f, 1.f }, 0.f);
    const cv::Matx33f C = drishti::geometry::ConicSection_<float>(E).getMatrix();
    const cv::Point3f L(1.f, 0.f, 0.f);

    cv::Vec3f P[2];
    const auto n = drishti::geometry::intersectConicLine(C, L, P);

    // [-0, 0.5, 1] [0, -0.5, 1]
    cv::Point2f p1(P[0][0] / P[0][2], P[0][1] / P[0][2]);
    cv::Point2f p2(P[1][0] / P[1][2], P[1][1] / P[1][2]);

    const float e1 = cv::norm(p1 - cv::Point2f(0.f, +0.5f));
    const float e2 = cv::norm(p2 - cv::Point2f(0.f, -0.5f));

    ASSERT_EQ(n, 2);
    ASSERT_LT(e1, 1e-6f);
    ASSERT_LT(e2, 1e-6f);

    // assertions on order, etc
}
