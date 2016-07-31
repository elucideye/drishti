/**
  @file   Image.hpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Top level Image declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the image structure used
  to store images that are processed by the top level SDK.
*/

#ifndef __drishtisdk__Image__
#define __drishtisdk__Image__

#include "drishti/drishti_sdk.hpp"
#include <cstdint>
#include <type_traits>
#include <stdlib.h>

_DRISHTI_SDK_BEGIN

/*
 * Vec types
 */

template <typename T, int D> struct Vec
{
public:
    Vec() {}

    template <typename... Tail>
    Vec(typename std::enable_if<sizeof...(Tail)+1 == D, T>::type head, Tail... tail) : val{ head, T(tail)... } {}

    T & operator [](int i)
    {
        return val[i];
    }
    const T & operator [](int i) const
    {
        return val[i];
    }
protected:
    T val[D];
};

typedef Vec<uint8_t, 3> Vec3b;
typedef Vec<float, 2> Vec2f;
typedef Vec<float, 3> Vec3f;
typedef Vec<int, 2> Vec2i;
typedef Vec<int, 3> Vec3i;

/*
 * Size2 types
 */

template < typename T >
struct Size2
{
    Size2() : width(0), height(0) {}
    Size2(T width, T height) : width(width), height(height) {}
    T width, height;
};

typedef Size2<float> Size2f;
typedef Size2<int> Size2i;

/*
 * Rect types
 */

template < typename T >
struct Rect
{
    Rect() : x(0), y(0), width(0), height(0) {}
    Rect(T x, T y, T width, T height) : x(x), y(y),  width(width), height(height) {}

    Size2<T> size() const
    {
        return Size2<T>(width, height);
    }

    T x, y, width, height;
};

typedef Rect<int> Recti;
typedef Rect<float> Rectf;

/*
 * Image types
 */


template <typename T> class DRISHTI_EXPORTS Image
{
public:
    Image();
    Image(const Image &src);
    Image(size_t rows, size_t cols, T *data, size_t stride, bool keep = false);
    ~Image();
    size_t getRows() const
    {
        return rows;
    }
    size_t getCols() const
    {
        return cols;
    }
    size_t getStride() const
    {
        return stride;    // bytes
    }
    template<typename T2> const T2* ptr() const
    {
        return reinterpret_cast<T2*>(data);
    }
    Image<T> clone();
protected:
    size_t rows = 0;
    size_t cols = 0;
    T *storage = 0, *data = 0;
    size_t stride = 0; // byte
};

typedef Image<uint8_t> Image1b;
typedef Image<Vec3b> Image3b;
typedef Image<float> Image1f;
typedef Image<Vec3f> Image3f;

_DRISHTI_SDK_END

#endif // __drishtisdk__Image__
