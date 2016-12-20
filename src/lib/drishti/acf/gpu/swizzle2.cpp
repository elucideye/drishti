/*!
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
    return TwoInputProc::useTexture(id, useTexUnit+position, target, position);
}

// *INDENT-OFF*
const char *MergeProc::fshaderMergeSrcABC1 = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif

 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;

 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);

     gl_FragColor = vec4(textureColor.rgb, textureColor2.r);
 });
// *INDENT-ON*

// *INDENT-OFF*
const char *MergeProc::fshaderMergeSrcAB12 = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif

 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;

 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);

     gl_FragColor = vec4(textureColor.rg, textureColor2.rg);
 });
// *INDENT-ON*

// *INDENT-OFF*
const char *MergeProc::fshaderMergeSrcAD12 = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 
 varying vec2 textureCoordinate;
 uniform sampler2D inputImageTexture;
 uniform sampler2D inputImageTexture2;
 
 void main()
 {
     vec4 textureColor = texture2D(inputImageTexture, textureCoordinate);
     vec4 textureColor2 = texture2D(inputImageTexture2, textureCoordinate);
     
     gl_FragColor = vec4(textureColor.ra, textureColor2.rg);
 });
// *INDENT-ON*

END_OGLES_GPGPU
