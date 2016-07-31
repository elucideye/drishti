/*!
  @file   gain.cpp
  @author David Hirvonen (C++ implementation) <dhirvonen elucideye com>
  @brief Implementation of ogles_gpgpu shader for gain.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "gain.h"

// *INDENT-OFF*
BEGIN_OGLES_GPGPU
const char * NoopProc::fshaderNoopSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 uniform float gain;
 void main()
 {
     vec4 val = texture2D(uInputTex, vTexCoord);
     gl_FragColor = clamp(val * gain, 0.0, 1.0);
 });
END_OGLES_GPGPU
// *INDENT-ON*
