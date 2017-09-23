/**
  @file   Image.cpp
  @author David Hirvonen
  @brief  Top level Image implementation

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the image structure used
  to store images that are processed by the top level SDK.
*/

#include "drishti/Image.hpp"

#include <memory>
#include <string.h> // cent os 6.2

_DRISHTI_SDK_BEGIN

template <typename T>
Image<T>::Image() {}

template <typename T>
Image<T>::~Image()
{
    if (storage)
    {
        delete[] storage;
    }
    storage = data = 0;
}

template <typename T>
Image<T>::Image(const Image& src)
    : rows(src.rows)
    , cols(src.cols)
    , storage(NULL)
    , data(src.data)
    , stride(src.stride)
{
}

template <typename T>
Image<T>::Image(size_t rows, size_t cols, T* data, size_t stride, bool keep)
    : rows(rows)
    , cols(cols)
    , storage(keep ? data : NULL)
    , data(data)
    , stride(stride)
{
}

template <typename T>
Image<T> Image<T>::clone()
{
    std::unique_ptr<T[]> d(new T[(rows * stride + sizeof(T) - 1) / sizeof(T)]);
    memcpy(d.get(), data, rows * stride);
    Image<T> dst(rows, cols, d.release(), stride);
    return dst;
}

template class DRISHTI_EXPORT Image<uint8_t>;
template class DRISHTI_EXPORT Image<Vec4b>;
template class DRISHTI_EXPORT Image<Vec3b>;
template class DRISHTI_EXPORT Image<float>;
template class DRISHTI_EXPORT Image<Vec3f>;

template struct DRISHTI_EXPORT Matrix<float, 3, 3>;

_DRISHTI_SDK_END
