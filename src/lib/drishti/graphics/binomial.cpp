/*! -*-c++-*-
  @file   binomial.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of an ogles_gpgpu binomial filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/binomial.h"

BEGIN_OGLES_GPGPU

// clang-format off
const char * BinomialProc::fshaderBinomialSrc = 
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision highp float;)
#endif
OG_TO_STR(
 varying vec2 textureCoordinate;
 varying vec2 leftTextureCoordinate;
 varying vec2 rightTextureCoordinate;

 varying vec2 topTextureCoordinate;
 varying vec2 topLeftTextureCoordinate;
 varying vec2 topRightTextureCoordinate;

 varying vec2 bottomTextureCoordinate;
 varying vec2 bottomLeftTextureCoordinate;
 varying vec2 bottomRightTextureCoordinate;

 uniform sampler2D inputImageTexture;

 void main()
{
    vec4 sw = texture2D(inputImageTexture, bottomLeftTextureCoordinate);
    vec4 ne = texture2D(inputImageTexture, topRightTextureCoordinate);
    vec4 nw = texture2D(inputImageTexture, topLeftTextureCoordinate);
    vec4 se = texture2D(inputImageTexture, bottomRightTextureCoordinate);
    vec4 w = texture2D(inputImageTexture, leftTextureCoordinate);
    vec4 c = texture2D(inputImageTexture, textureCoordinate);
    vec4 e = texture2D(inputImageTexture, rightTextureCoordinate);
    vec4 s = texture2D(inputImageTexture, bottomTextureCoordinate);
    vec4 n = texture2D(inputImageTexture, topTextureCoordinate);

    vec4 final = (sw + ne + nw + se) / 16.0 + (n + s + w + e) / 8.0 + c / 4.0;

    gl_FragColor = final;
});
END_OGLES_GPGPU
// clang-format on
