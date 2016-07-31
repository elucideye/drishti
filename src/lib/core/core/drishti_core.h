/*!
  @file   drishti_core.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of internal drishti core namespace.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_CORE_H
#define DRISHTI_CORE_H

#define DRISHTI_BEGIN_NAMESPACE(X) namespace X {
#define DRISHTI_END_NAMESPACE(X) }

#define _DRISHTI_BEGIN namespace drishti {
#define _DRISHTI_END }

#define DRISHTI_CORE_BEGIN namespace drishti { namespace core {
#define DRISHTI_CORE_END } }

#include <memory>
#include <vector>
#include <numeric>
#include <algorithm>
#include <cmath>

DRISHTI_CORE_BEGIN

// http://clean-cpp.org/underprivileged-unique-pointers-make_unique/#more-54
template <typename Value, typename ... Arguments>
std::unique_ptr<Value> make_unique(Arguments && ... arguments_for_constructor)
{
    return std::unique_ptr<Value>(new Value(std::forward<Arguments>(arguments_for_constructor)...));
}

template <typename T, typename Comp=std::less<T> >
std::vector<size_t> ordered(const std::vector<T> & values, const Comp &C)
{
    std::vector<size_t> indices(values.size());
    std::iota(std::begin(indices), std::end(indices), static_cast<size_t>(0));
    std::sort(std::begin(indices), std::end(indices),[&](size_t a, size_t b)
    {
        return C(values[a], values[b]);
    });
    return indices;
}

template<typename T> T logN(const T &x, const T &n)
{
    return std::log(x)/ std::log(n);
}
template<typename T> T log2(const T &x)
{
    return logN(x, T(2));
}
template<typename T> T round(T x);

DRISHTI_CORE_END


#endif
