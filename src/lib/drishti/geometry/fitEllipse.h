/*!
  @file   fitEllipse.h
  @author David Hirvonen
  @brief  Declaration of ellipse fitting routines.

  Lineage: 
  http://research.microsoft.com/en-us/um/people/awf/ellipse/fitellipse.html
  Copyright (c) 1999, Andrew Fitzgibbon, Maurizio Pilu, Bob Fisher  

*/

#ifndef __drishti_geometry_fitEllipse_h__
#define __drishti_geometry_fitEllipse_h__ 1

#include "drishti/geometry/drishti_geometry.h"

#include <opencv2/core.hpp>

#include <vector>

DRISHTI_GEOMETRY_BEGIN

template <typename T> T sign(T A)
{
    return T(int(A > 0) - int(A < 0));
}

template <typename T> T pow2(const T&x)
{
    return x*x;
}
cv::RotatedRect conicPar2Cen(const cv::Vec6d &par);
cv::Vec6d conicCen2Par(const cv::RotatedRect &cen);

#if !DRISHTI_BUILD_MIN_SIZE
cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &pts);
cv::RotatedRect fitEllipse(const std::vector<cv::Point2d> &points, const cv::Point2d &center);
#endif // !DRISHTI_BUILD_MIN_SIZE

DRISHTI_GEOMETRY_END

#endif // d__drishti_geometry_fitEllipse_h__ 1
