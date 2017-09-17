/*! -*-c++-*-
  @file   Rectangle.h
  @author David Hirvonen
  @brief  Declaration of rectangle class and geometric operations.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_Rectangle_h__
#define __drishti_geometry_Rectangle_h__

#include "drishti/geometry/drishti_geometry.h"
#include "drishti/geometry/Ellipse.h"
#include <opencv2/core/core.hpp>
#include <vector>
#include <array>

DRISHTI_GEOMETRY_BEGIN

template <typename T1, typename T2>
inline cv::Rect_<T2> operator*(const cv::Matx<T1, 3, 3>& H, const cv::Rect_<T2>& src)
{
    // Transform the roi:
    cv::Point_<T1> tl = src.tl();
    cv::Point3_<T1> tl_ = H * cv::Point3_<T1>(tl.x, tl.y, 1.f);
    cv::Point_<T1> br = src.br();
    cv::Point3_<T1> br_ = H * cv::Point3_<T1>(br.x, br.y, 1.f);
    cv::Rect_<T2> dst(cv::Point_<T2>(tl_.x / tl_.z, tl_.y / tl_.z), cv::Point_<T2>(br_.x / br_.z, br_.y / br_.z));
    return dst;
}

template <typename T1, typename T2, typename T3 = double>
inline cv::Point_<T2> centroid(const cv::Rect_<T1>& roi)
{
    cv::Point_<T3> tl = roi.tl(), br = roi.br(), center = (tl + br) / T3(2);
    cv::Point_<T2> center_ = center; // use built in round
    return center_;
}
DRISHTI_GEOMETRY_END

#endif /* defined(__drishti_geometry_Rectangle_h__) */
