/*! -*-c++-*-
  @file   arithmetic.h
  @author David Hirvonen
  @brief  Declaration of optimized vector arithmetic.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_arithmetic_h__
#define __drishti_core_arithmetic_h__

#include "drishti/core/drishti_core.h"

#include <cstdint>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename T>
T round(T x);

void add16sAnd16s(const int16_t* pa, const int16_t* pb, int16_t* pc, int n);
void add16sAnd32s(const int32_t* pa, const int16_t* pb, int32_t* pc, int n);
void add32f(const float* pa, const float* pb, float* pc, int n);
void convertFixedPoint(const float* pa, int16_t* pb, int n, int fraction);

DRISHTI_CORE_NAMESPACE_END

#endif
