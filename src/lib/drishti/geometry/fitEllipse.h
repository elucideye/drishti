/*! -*-c++-*-
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

template <typename T>
T sign(T A)
{
    return T(int(A > 0) - int(A < 0));
}

template <typename T>
T pow2(const T& x)
{
    return x * x;
}
cv::RotatedRect conicPar2Cen(const cv::Vec6d& par);
cv::Vec6d conicCen2Par(const cv::RotatedRect& cen);

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_fitEllipse_h__ 1
