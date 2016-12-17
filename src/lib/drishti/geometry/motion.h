/*!
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
#define DRISHTI_TRANSFORMATION_NAMESPACE_BEGIN namespace transformation  {
#define DRISHTI_TRANSFORMATION_NAMESPACE_END }

DRISHTI_TRANSFORMATION_NAMESPACE_BEGIN

inline cv::Point2f center(const cv::Rect &roi)
{
    cv::Point2f tl = roi.tl(), br = roi.br(), center = (tl + br) * 0.5f;
    return center;
}

template <typename T>
cv::Point_<T> dehomogenize(const cv::Point3_<T> &p)
{
    return cv::Point_<T>(p.x/p.z, p.y/p.z);
}

cv::Matx33f estimateSimilarity(const std::array<cv::Point2f,2> &p, const std::array<cv::Point2f,2> &q);

cv::Mat estimateGlobMotionLeastSquaresSimilarity(int npoints, cv::Point2f *points0, cv::Point2f *points1, float *rmse);

inline cv::Matx33f translate(float x, float y)
{
    return cv::Matx33f(1,0,x,0,1,y,0,0,1);
}

inline cv::Matx33f scale(float x, float y)
{
    return cv::Matx33f(cv::Matx33f::diag({x,y,1.f}));
}

inline cv::Matx33f scale(float x, float y, const cv::Point2f &c)
{
    return translate(+c.x,+c.y) * scale(x,y) * translate(-c.x,-c.y);
}

inline cv::Matx33f transpose()
{
    return cv::Matx33f(0,1,0,1,0,0,0,0,1);
}
inline cv::Matx33f rotate(float theta)
{
    const float a = std::cos(theta);
    const float b = std::sin(theta);
    return cv::Matx33f(a, -b, 0, b, a, 0, 0, 0, 1);
}

inline cv::Matx33f rotate(int degrees)
{
    switch(degrees)
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
            return  scale(+1.f, -1.f) * transpose();
        default :
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
void R3x3To4x4(const cv::Matx33f &R3, cv::Matx44f &R4);

inline cv::Matx33f normalize(const cv::Size &sizeIn)
{
    const cv::Point center(sizeIn.width/2, sizeIn.height/2);
    cv::Matx33f T = transformation::translate(-center.x, -center.y);
    cv::Matx33f S = transformation::scale(2.0 / float(sizeIn.width), 2.0 / float(sizeIn.height));
    return S * T;
}

inline cv::Matx33f denormalize(const cv::Size &sizeIn)
{
    const cv::Point frameCenter(sizeIn.width/2, sizeIn.height/2);
    cv::Matx33f T = transformation::translate(1.0, 1.0);
    cv::Matx33f S = transformation::scale(float(sizeIn.width)/2.0, float(sizeIn.height)/2.0);
    return S * T;
}

DRISHTI_TRANSFORMATION_NAMESPACE_END

#endif // __drishti_geometry_motion_h__
