/*! -*-c++-*-
  @file   Vector1d.h
  @author David Hirvonen
  @brief  Declaration of utility vector class and common arithmetic operations.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_Vector1d_h__
#define __drishti_rcpr_Vector1d_h__ 1

#include "drishti/rcpr/drishti_rcpr.h"

DRISHTI_RCPR_NAMESPACE_BEGIN

inline cv::RotatedRect operator*(const cv::RotatedRect& e, float scale)
{
    return cv::RotatedRect(e.center * scale, e.size * scale, e.angle);
}

//

template <typename T>
std::vector<T> operator*(const std::vector<T>& a, const T& b)
{
    std::vector<T> c(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        c[i] = a[i] * b;
    }
    return c;
}

template <typename T>
std::vector<T> operator+(const std::vector<T>& a, const std::vector<T>& b)
{
    std::vector<T> c(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        c[i] = a[i] + b[i];
    }

    return c;
}

template <typename T>
std::vector<T> operator-(const std::vector<T>& a, const std::vector<T>& b)
{
    std::vector<T> c(a.size());
    for (int i = 0; i < a.size(); i++)
    {
        c[i] = a[i] - b[i];
    }

    return c;
}

template <typename T>
std::vector<T>& operator+=(std::vector<T>& a, const std::vector<T>& b)
{
    for (int i = 0; i < a.size(); i++)
    {
        a[i] += b[i];
    }

    return a;
}

template <typename T>
std::vector<T>& operator*=(std::vector<T>& a, const T& value)
{
    for (int i = 0; i < a.size(); i++)
    {
        a[i] *= value;
    }

    return a;
}

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_Vector1d_h__
