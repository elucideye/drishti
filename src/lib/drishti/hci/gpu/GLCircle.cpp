/*! -*-c++-*-
  @file GLCircle.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of ogles_gpgpu shader for rendering circles.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/GLCircle.h"

// clang-format off
BEGIN_OGLES_GPGPU
const char * CircleProc::fshaderCircleSrc =
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision mediump float;)
#endif
OG_TO_STR(
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 uniform vec3 color; 
 uniform vec2 center;
 uniform float radius;
 uniform float ratio;
 void main()
 {
     vec2 delta = vTexCoord - center;
     float d = step(length(vec2(delta.x * ratio, delta.y)), radius);
     vec4 val = texture2D(uInputTex, vTexCoord);
     gl_FragColor = vec4(d * color + (1.0 - d) * val.rgb, val.a);
 });
END_OGLES_GPGPU
// clang-format on
