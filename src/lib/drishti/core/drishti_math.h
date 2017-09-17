/*! -*-c++-*-
  @file   drishti_math.h
  @author David Hirvonen
  @brief  Declaration of common math routines.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_math_h__
#define __drishti_core_drishti_math_h__ 1

#include "drishti/core/drishti_core.h"

#include <cmath>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename T>
T logN(const T& x, const T& n)
{
    return std::log(x) / std::log(n);
}
template <typename T>
T log2(const T& x)
{
    return logN(x, T(2));
}
template <typename T>
T round(T x);

DRISHTI_CORE_NAMESPACE_END

#endif // drishti_math.h
