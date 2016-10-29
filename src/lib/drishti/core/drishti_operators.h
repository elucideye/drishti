/*!
  @file   drishti_operator.h
  @author David Hirvonen
  @brief  Declaration of common arithmetic operators.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_CORE_OPERATOR_H
#define DRISHTI_CORE_OPERATOR_H 1

#include "drishti/core/drishti_core.h"

#include <opencv2/core/core.hpp>

DRISHTI_CORE_NAMESPACE_BEGIN

// Local template definitions:
template <typename T1, typename T2>
cv::Rect operator *(const cv::Rect_<T1> &roi, const T2 &scale)
{
    return cv::Rect_<T1>(roi.x * scale, roi.y * scale, roi.width * scale, roi.height * scale);
}

template <typename T1, typename T2>
std::vector<T1> operator *(const std::vector<T1> &src, const T2 &scale)
{
    auto dst = src;
    for(auto &r : dst)
    {
        r = r * scale;
    }
    return dst;
}

template <typename T1, typename T2>
cv::Size_<T1> operator*(const cv::Size_<T1> &size, const T2 &scale)
{
    return cv::Size_<T1>(T2(size.width) * scale, T2(size.height) * scale);
}

DRISHTI_CORE_NAMESPACE_END

#endif // DRISHTI_CORE_OPERATOR_H
