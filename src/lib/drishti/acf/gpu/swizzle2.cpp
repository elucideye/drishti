/*! -*-c++-*-
  @file   swizzle2.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of an ogles_gpgpu simple triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/gpu/swizzle2.h"

BEGIN_OGLES_GPGPU

/**
 * Perform a standard shader initialization.
 */
int MergeProc::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    return FilterProcBase::init(inW, inH, order, prepareForExternalInput);
}

void MergeProc::useTexture(GLuint id, GLuint useTexUnit, GLenum target, int position)
{
    return TwoInputProc::useTexture(id, useTexUnit + position, target, position);
}

// clang-format off
const char *MergeProc::fshaderMergeSrcABC1 = 
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
OG_TO_STR(
 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;

 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);

     gl_FragColor = vec4(textureColor.rgb, textureColor2.r);
 });
// clang-format on

// clang-format off
const char *MergeProc::fshaderMergeSrcAB12 = 
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
OG_TO_STR(
 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;

 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);

     gl_FragColor = vec4(textureColor.rg, textureColor2.rg);
 });
// clang-format on

// clang-format off
const char *MergeProc::fshaderMergeSrcAD12 = 
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
OG_TO_STR(
 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;
 
 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);
     
     gl_FragColor = vec4(textureColor.ra, textureColor2.rg);
 });
// clang-format on

END_OGLES_GPGPU
