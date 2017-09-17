/*! -*-c++-*-
  @file   convert.h
  @author David Hirvonen
  @brief  Declaration of optimized unpack and format conversion routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_convert_h__
#define __drishti_core_convert_h__ 1

#include "drishti/core/drishti_core.h"
#include <opencv2/core/core.hpp>

DRISHTI_CORE_NAMESPACE_BEGIN

struct PlaneInfo
{
    PlaneInfo(cv::Mat& plane, int channel = 0, float alpha = 1.f)
        : plane(plane)
        , channel(channel)
        , alpha(alpha)
    {
    }
    cv::Mat plane;
    int channel = 0;
    float alpha = 1.f;
};

void convertU8ToF32(const cv::Mat4b& input, std::vector<PlaneInfo>& planes);

void unpack(const cv::Mat4b& input, std::vector<PlaneInfo>& planes);

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_convert_h__
