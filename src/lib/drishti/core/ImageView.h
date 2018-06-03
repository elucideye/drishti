/*! -*-c++-*-
  @file   drishti/core/ImageView.h
  @author David Hirvonen
  @brief Container class for multiple views of the same image (RAM, GPU)

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_ImageView_h__
#define __drishti_core_ImageView_h__

#include <opencv2/core.hpp> // for cv::Mat4b
#include <utility>

DRISHTI_CORE_NAMESPACE_BEGIN

struct Texture
{
    Texture() = default;
    Texture(const cv::Size& size, std::uint32_t texId)
        : size(size)
        , texId(texId)
    {
    }
    cv::Size size;           //! Teture dimensions in pixels
    std::uint32_t texId = 0; //! Identifier for the texture
};

struct ImageView
{
    ImageView() = default;
    ImageView(Texture  texture, cv::Mat4b  image)
        : texture(std::move(texture))
        , image(std::move(image))
    {
    }
    Texture texture; //! Texture descriptor
    cv::Mat4b image; //! Image descriptor
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_ImageView_h__
