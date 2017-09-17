/*! -*-c++-*-
  @file   geometry/motion.cpp
  @author David Hirvonen
  @brief  Shared routines for expression parametric motion (homogeneous coordinates).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/geometry/motion.h"

//============================
//== Utility =================
//============================

DRISHTI_TRANSFORMATION_NAMESPACE_BEGIN

// OpenCV 3-Clause BSD
// https://github.com/opencv/opencv/blob/21ee113af30a2efba8faac4811d55822ff878b0e/modules/videostab/src/global_motion.cpp#L85
static cv::Mat normalizePoints(int npoints, cv::Point2f* points)
{
    float cx = 0.f, cy = 0.f;
    for (int i = 0; i < npoints; ++i)
    {
        cx += points[i].x;
        cy += points[i].y;
    }
    cx /= npoints;
    cy /= npoints;

    float d = 0.f;
    for (int i = 0; i < npoints; ++i)
    {
        points[i].x -= cx;
        points[i].y -= cy;
        d += std::sqrt(points[i].x * points[i].x + points[i].y * points[i].y);
    }
    d /= npoints;

    float s = std::sqrt(2.f) / d;
    for (int i = 0; i < npoints; ++i)
    {
        points[i].x *= s;
        points[i].y *= s;
    }

    cv::Mat_<float> T = cv::Mat::eye(3, 3, CV_32F);
    T(0, 0) = T(1, 1) = s;
    T(0, 2) = -cx * s;
    T(1, 2) = -cy * s;
    return T;
}

// OpenCV 3-Clause BSD
// https://github.com/opencv/opencv/blob/21ee113af30a2efba8faac4811d55822ff878b0e/modules/videostab/src/global_motion.cpp#L280
cv::Mat estimateGlobMotionLeastSquaresSimilarity(int npoints, cv::Point2f* points0, cv::Point2f* points1, float* rmse)
{
    cv::Mat_<float> T0 = normalizePoints(npoints, points0);
    cv::Mat_<float> T1 = normalizePoints(npoints, points1);

    cv::Mat_<float> A(2 * npoints, 4), b(2 * npoints, 1);
    float *a0, *a1;
    cv::Point2f p0, p1;

    for (int i = 0; i < npoints; ++i)
    {
        a0 = A[2 * i];
        a1 = A[2 * i + 1];
        p0 = points0[i];
        p1 = points1[i];
        a0[0] = p0.x;
        a0[1] = p0.y;
        a0[2] = 1;
        a0[3] = 0;
        a1[0] = p0.y;
        a1[1] = -p0.x;
        a1[2] = 0;
        a1[3] = 1;
        b(2 * i, 0) = p1.x;
        b(2 * i + 1, 0) = p1.y;
    }

    cv::Mat_<float> sol;
    cv::solve(A, b, sol, cv::DECOMP_NORMAL | cv::DECOMP_LU);

    if (rmse)
    {
        *rmse = static_cast<float>(norm(A * sol, b, cv::NORM_L2) / std::sqrt(static_cast<double>(npoints)));
    }

    cv::Mat_<float> M = cv::Mat::eye(3, 3, CV_32F);
    M(0, 0) = M(1, 1) = sol(0, 0);
    M(0, 1) = sol(1, 0);
    M(1, 0) = -sol(1, 0);
    M(0, 2) = sol(2, 0);
    M(1, 2) = sol(3, 0);

    return T1.inv() * M * T0;
}

cv::Matx33f estimateSimilarity(const std::array<cv::Point2f, 2>& p, const std::array<cv::Point2f, 2>& q)
{
    const cv::Point2f v1 = (p[0] - p[1]);
    const cv::Point2f v2 = (q[0] - q[1]);
    const cv::Point2f c1 = (p[0] + p[1]) * 0.5f;
    const cv::Point2f c2 = (q[0] + q[1]) * 0.5f;
    const float d1 = cv::norm(v1);
    const float d2 = cv::norm(v2);
    const float s = d2 / d1;

    // Signed angle between two normalized vectors
    const float theta = std::atan2(v1.cross(v2), v1.dot(v2));

    const cv::Matx33f R = rotate(theta);
    const cv::Matx33f S = scale(s, s);
    const cv::Matx33f T1 = translate(-c1.x, -c1.y);
    const cv::Matx33f T2 = translate(+c2.x, +c2.y);
    const cv::Matx33f H = T2 * S * R * T1;

    return H;
}

// Map 3x3 homography to 4x4 matrix:
//
// H00 H01 H02      H00 H01 0 H02
// H10 H11 H12  =>  H10 H11 0 H12
// H20 H21 H22       0   0  1  0
//                  H20 H21 0 H22
void R3x3To4x4(const cv::Matx33f& R3, cv::Matx44f& R4)
{
    R4 = cv::Matx44f::eye();

    R4(0, 0) = R3(0, 0);
    R4(0, 1) = R3(0, 1);
    R4(0, 3) = R3(0, 2);

    R4(1, 0) = R3(1, 0);
    R4(1, 1) = R3(1, 1);
    R4(1, 3) = R3(1, 2);

    R4(3, 0) = R3(2, 0);
    R4(3, 1) = R3(2, 1);
    R4(3, 3) = R3(2, 2);
}

DRISHTI_TRANSFORMATION_NAMESPACE_END
