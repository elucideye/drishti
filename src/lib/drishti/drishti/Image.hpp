/**
  @file   Image.hpp
  @author David Hirvonen
  @brief  Top level Image declaration.

  \copyright Copyright 2014-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the image structure used
  to store images that are processed by the top level SDK.
*/

#ifndef __drishti_drishti_Image_hpp__
#define __drishti_drishti_Image_hpp__

#include <drishti/drishti_sdk.hpp>
#include <cstdint>
#include <type_traits>
#include <cstdlib>

_DRISHTI_SDK_BEGIN

/*
 * Vec types
 */

template <typename T, int D>
struct Vec
{
public:
    Vec() = default;

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

using Vec3b = Vec<uint8_t, 3>;
using Vec4b = Vec<uint8_t, 4>;
using Vec2f = Vec<float, 2>;
using Vec3f = Vec<float, 3>;
using Vec2i = Vec<int, 2>;
using Vec3i = Vec<int, 3>;

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

using Size2f = Size2<float>;
using Size2i = Size2<int>;

/*
 * Matrix types
 */

template <typename T, int rowDim, int colDim>
struct Matrix
{
    Matrix() = default;
    ~Matrix() = default;
    Matrix(const Matrix& src)
    {
        for (int y = 0; y < rowDim; y++)
        {
            for (int x = 0; x < colDim; x++)
            {
                data[y][x] = src(y, x);
            }
        }
    }

    Matrix(Matrix&&) noexcept = default;
    Matrix& operator=(const Matrix&) = delete;
    Matrix& operator=(Matrix&&) = delete;

    int rows() const { return rowDim; }
    int cols() const { return colDim; }

    T& operator()(int y, int x) { return data[y][x]; }
    const T& operator()(int y, int x) const { return data[y][x]; }

    static Matrix<T, rowDim, colDim> eye()
    {
        Matrix<T, rowDim, colDim> I{};
        for (int y = 0; y < 3; y++)
        {
            for (int x = 0; x < 3; x++)
            {
                I(y, x) = 0.f;
            }
            I(y, y) = 1.f;
        }
        return I;
    }

    T data[rowDim][colDim];
};

using Matrix33f = Matrix<float, 3, 3>;

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

using Recti = Rect<int>;
using Rectf = Rect<float>;

/*
 * Texture container
 */

struct DRISHTI_EXPORT Texture
{
    Texture() = default;
    Texture(const Vec2i& size, std::uint32_t texId)
        : size(size)
        , texId(texId)
    {
        // copy
    }

    Vec2i size{};
    std::uint32_t texId{};
};

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

    Image(Image&&) noexcept = default;
    Image& operator=(const Image&) = default;
    Image& operator=(Image&&) noexcept = default;

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
        return reinterpret_cast<T2*>(data); // NOLINT (TODO)
    }
    Image<T> clone();

protected:
    size_t rows = 0;
    size_t cols = 0;
    T *storage = nullptr, *data = nullptr;
    size_t stride = 0; // byte
};

using Image1b = Image<uint8_t>;
using Image3b = Image<Vec3b>;
using Image4b = Image<Vec4b>;
using Image1f = Image<float>;
using Image3f = Image<Vec3f>;

_DRISHTI_SDK_END

#endif // __drishti_drishti_Image_hpp__
