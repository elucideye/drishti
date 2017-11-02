/*! -*-c++-*-
  @file   saturation.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of ogles_gpgpu shader for saturation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}
 
  Compute saturation measure as euclidean distance from white in RGB
  Using (euclidean) LUV colorspace may provide more satisfactory results.dh

*/

#include "drishti/graphics/saturation.h"

// clang-format off
BEGIN_OGLES_GPGPU
const char * SaturationProc::fshaderSaturationSrc = 
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision mediump float;)
#endif
OG_TO_STR(
 const float sqrt3 = 1.7320508075;
 
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 uniform float gain;
 void main()
 {
     vec4 val = texture2D(uInputTex, vTexCoord);
     float d = 1.0 - (distance(val.rgb, vec3(1.0,1.0,1.0)) / sqrt3);
     vec3 d3 = clamp(vec3(d,d,d) * gain, 0.0, 1.0);
     gl_FragColor = vec4(d3, 1.0);
 });
END_OGLES_GPGPU
// clang-format on
