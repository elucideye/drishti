/**
  @file   Image.hpp
  @author David Hirvonen
  @brief  Top level Image declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the image structure used
  to store images that are processed by the top level SDK.
*/

#ifndef __drishti_drishti_Image_hpp__
#define __drishti_drishti_Image_hpp__

#include "drishti/drishti_sdk.hpp"
#include <cstdint>
#include <type_traits>
#include <stdlib.h>

_DRISHTI_SDK_BEGIN

/*
 * Vec types
 */

template <typename T, int D>
struct Vec
{
public:
    Vec() {}

    template <typename... Tail>
    Vec(typename std::enable_if<sizeof...(Tail) + 1 == D, T>::type head, Tail... tail)
        : val{ head, T(tail)... }
    {
    }

    T& operator[](int i)
    {
        return val[i];
    }
    const T& operator[](int i) const
    {
        return val[i];
    }

protected:
    T val[D];
};

typedef Vec<uint8_t, 3> Vec3b;
typedef Vec<uint8_t, 4> Vec4b;
typedef Vec<float, 2> Vec2f;
typedef Vec<float, 3> Vec3f;
typedef Vec<int, 2> Vec2i;
typedef Vec<int, 3> Vec3i;

/*
 * Size2 types
 */

template <typename T>
struct Size2
{
    Size2()
        : width(0)
        , height(0)
    {
    }
    Size2(T width, T height)
        : width(width)
        , height(height)
    {
    }
    T width, height;
};

typedef Size2<float> Size2f;
typedef Size2<int> Size2i;

/*
 * Matrix types
 */

template <typename T, int rowDim, int colDim>
struct Matrix
{
    Matrix() {}
    Matrix(const Matrix &src)
    {
        for (int y = 0; y < rowDim; y++)
        {
            for (int x = 0; x< colDim; x++)
            {
                data[y][x] = src(y,x);
            }
        }
    }

    int rows() const { return rowDim; }
    int cols() const { return colDim; }
    
    T& operator()(int y, int x) { return data[y][x]; }
    const T& operator()(int y, int x) const { return data[y][x]; }

    static Matrix<T, rowDim, colDim> eye()
    {
        Matrix<T, rowDim, colDim> I;
        for (int y = 0; y < 3; y++)
        {
            for (int x = 0; x < 3; x++)
            {
                I(y,x) = 0.f;
            }
            I(y,y) = 1.f;
        }
        return I;
    }
    
    T data[rowDim][colDim];
};

typedef Matrix<float, 3, 3> Matrix33f;

/*
 * Rect types
 */

template <typename T>
struct Rect
{
    Rect()
        : x(0)
        , y(0)
        , width(0)
        , height(0)
    {
    }
    Rect(T x, T y, T width, T height)
        : x(x)
        , y(y)
        , width(width)
        , height(height)
    {
    }

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

template <typename T>
class DRISHTI_EXPORT Image
{
public:
    Image();
    Image(const Image& src);
    Image(size_t rows, size_t cols, T* data, size_t stride, bool keep = false);
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
        return stride; // bytes
    }
    template <typename T2>
    const T2* ptr() const
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
typedef Image<Vec4b> Image4b;
typedef Image<float> Image1f;
typedef Image<Vec3f> Image3f;

_DRISHTI_SDK_END

#endif // __drishti_drishti_Image_hpp__
