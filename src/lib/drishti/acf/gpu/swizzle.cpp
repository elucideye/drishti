/*!
  @file   swizzle.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of an ogles_gpgpu simple triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/gpu/swizzle.h"

// *INDENT-OFF*
BEGIN_OGLES_GPGPU
const char * SwizzleProc::fshaderBGRASrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 void main()
 {
     vec4 val = texture2D(uInputTex, vTexCoord);
     gl_FragColor = val.bgra;
 });
END_OGLES_GPGPU
// *INDENT-ON*
