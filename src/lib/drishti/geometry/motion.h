/*! -*-c++-*-
  @file   geometry/motion.h
  @author David Hirvonen
  @brief  Shared routines for expression parametric motion (homogeneous coordinates).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_geometry_motion_h__
#define __drishti_geometry_motion_h__

#include <opencv2/core.hpp>
#include <array>

// Define common rotation operations with cv::Matx33f

// clang-format off
#define DRISHTI_TRANSFORMATION_NAMESPACE_BEGIN  namespace transformation {
#define DRISHTI_TRANSFORMATION_NAMESPACE_END }
// clang-format on

DRISHTI_TRANSFORMATION_NAMESPACE_BEGIN

inline cv::Point2f center(const cv::Rect& roi)
{
    cv::Point2f tl = roi.tl(), br = roi.br(), center = (tl + br) * 0.5f;
    return center;
}

template <typename T>
cv::Point_<T> dehomogenize(const cv::Point3_<T>& p)
{
    return cv::Point_<T>(p.x / p.z, p.y / p.z);
}

cv::Matx33f estimateSimilarity(const std::array<cv::Point2f, 2>& p, const std::array<cv::Point2f, 2>& q);

cv::Mat estimateGlobMotionLeastSquaresSimilarity(int npoints, cv::Point2f* points0, cv::Point2f* points1, float* rmse);

inline cv::Matx33f translate(float x, float y)
{
    return cv::Matx33f(1, 0, x, 0, 1, y, 0, 0, 1);
}

inline cv::Matx33f translate(const cv::Point2f& t)
{
    return translate(t.x, t.y);
}

inline cv::Matx33f scale(float x, float y)
{
    return cv::Matx33f(cv::Matx33f::diag({ x, y, 1.f }));
}

inline cv::Matx33f scale(float s)
{
    return scale(s, s);
}

inline cv::Matx33f scale(float x, float y, const cv::Point2f& c)
{
    return translate(+c.x, +c.y) * scale(x, y) * translate(-c.x, -c.y);
}

inline cv::Matx33f transpose()
{
    return cv::Matx33f(0, 1, 0, 1, 0, 0, 0, 0, 1);
}
inline cv::Matx33f rotate(float theta)
{
    const float a = std::cos(theta);
    const float b = std::sin(theta);
    return cv::Matx33f(a, -b, 0, b, a, 0, 0, 0, 1);
}

inline cv::Matx33f rotate(int degrees)
{
    switch (degrees)
    {
        case 360:
        case 0:
            return cv::Matx33f::eye();
            break;
        case 90:
            return scale(-1.f, 1.f) * transpose();
        case 180:
            return scale(1.f, -1.f);
        case 270:
            return scale(+1.f, -1.f) * transpose();
        default:
            assert(false);
    }
    return cv::Matx33f::eye();
}

// Map 3x3 homography to 4x4 matrix:
//
// H00 H01 H02      H00 H01 0 H02
// H10 H11 H12  =>  H10 H11 0 H12
// H20 H21 H22       0   0  1  0
//                  H20 H21 0 H22
void R3x3To4x4(const cv::Matx33f& R3, cv::Matx44f& R4);

// For OpenGL textures:
inline cv::Matx33f normalize(const cv::Size& sizeIn)
{
    const cv::Point center(sizeIn.width / 2, sizeIn.height / 2);
    cv::Matx33f T = transformation::translate(-center.x, -center.y);
    cv::Matx33f S = transformation::scale(2.0f / float(sizeIn.width), 2.0f / float(sizeIn.height));
    return S * T;
}

inline cv::Matx33f denormalize(const cv::Size& sizeIn)
{
    const cv::Point frameCenter(sizeIn.width / 2, sizeIn.height / 2);
    cv::Matx33f T = transformation::translate(1.f, 1.f);
    cv::Matx33f S = transformation::scale(float(sizeIn.width) / 2.f, float(sizeIn.height) / 2.f);
    return S * T;
}

inline cv::Point2f normalize(const cv::Point2f& p, const cv::Size2f& sizeIn)
{
    return cv::Point2f(p.x / sizeIn.width, p.y / sizeIn.height);
}

inline cv::Point2f denormalize(const cv::Point2f& p, const cv::Size2f& sizeIn)
{
    return cv::Point2f(p.x * sizeIn.width, p.y * sizeIn.height);
}

// https://www.opengl.org/sdk/docs/man2/xhtml/glOrtho.xml
inline cv::Matx44f glOrtho(float left, float right, float bottom, float top, float nearP, float farP)
{
    cv::Matx44f P = cv::Matx44f::eye();
    P(0, 0) = 2.0f / (right - left);
    P(0, 3) = -((right + left) / (right - left));
    P(1, 1) = 2.0f / (top - bottom);
    P(1, 3) = -((top + bottom) / (top - bottom));
    P(2, 2) = -(2.0f / (farP - nearP));
    P(2, 3) = -((farP + nearP) / (farP - nearP));
    return P;
}

// https://www.opengl.org/sdk/docs/man2/xhtml/gluPerspective.xml
inline cv::Matx44f glPerspective(float fov, float ratio, float nearP, float farP)
{
    cv::Matx44f P = cv::Matx44f::eye();
    const float f = 1.0f / std::tan(fov * (M_PI / 360.0));
    P(0, 0) = f / ratio;
    P(1, 1) = f;
    P(2, 2) = (farP + nearP) / (nearP - farP);
    P(2, 3) = (2.0f * farP * nearP) / (nearP - farP);
    P(3, 2) = -1.0f;
    P(3, 3) = 0.0f;
    return P;
}

DRISHTI_TRANSFORMATION_NAMESPACE_END

#endif // __drishti_geometry_motion_h__
