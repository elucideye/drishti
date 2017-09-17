/*! -*-c++-*-
  @file   drishti_algorithm.h
  @author David Hirvonen
  @brief  Return vector index for sorted array.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_algorithm_h__
#define __drishti_core_drishti_algorithm_h__ 1

#include <vector>
#include <numeric>
#include <algorithm>

DRISHTI_CORE_NAMESPACE_BEGIN

template <typename T, typename Comp = std::less<T>>
std::vector<size_t> ordered(const std::vector<T>& values, const Comp& C)
{
    std::vector<size_t> indices(values.size());
    std::iota(std::begin(indices), std::end(indices), static_cast<size_t>(0));
    std::sort(std::begin(indices), std::end(indices), [&](size_t a, size_t b) {
        return C(values[a], values[b]);
    });
    return indices;
}

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_drishti_algorithm_h__ 1
