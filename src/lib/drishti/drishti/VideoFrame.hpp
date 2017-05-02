/**
  @file   VideoFrame.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the VideoFrame public API class.
*/

#ifndef __drishti_drishti_VideoFrame_hpp__
#define __drishti_drishti_VideoFrame_hpp__

#include "drishti/drishti_sdk.hpp"
#include "drishti/drishti_gl.hpp"
#include "drishti/Image.hpp"

_DRISHTI_SDK_BEGIN

struct DRISHTI_EXPORT VideoFrame
{
    VideoFrame() {}
    VideoFrame(const Vec2i& size, void* pixelBuffer, bool useRawPixels, GLuint inputTexture, GLenum textureFormat)
        : size(size)
        , pixelBuffer(pixelBuffer)
        , useRawPixels(useRawPixels)
        , inputTexture(inputTexture)
        , textureFormat(textureFormat)
    {
    }

    Vec2i size;
    void* pixelBuffer = nullptr;
    bool useRawPixels = false;
    GLuint inputTexture = 0;
    GLenum textureFormat = 0;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_VideoFrame_hpp__
