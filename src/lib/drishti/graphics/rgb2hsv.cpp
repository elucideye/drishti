/*! -*-c++-*-
  @file   rgb2hsv.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of ogles_gpgpu shader for rgb2hsv.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/rgb2hsv.h"

// clang-format off
BEGIN_OGLES_GPGPU
const char * Rgb2HsvProc::fshaderRgb2HsvSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif

 vec3 rgb2hsv(vec3 c)
 {
    vec4 K = vec4(0.0, -1.0 / 3.0, 2.0 / 3.0, -1.0);
    vec4 p = mix(vec4(c.bg, K.wz), vec4(c.gb, K.xy), step(c.b, c.g));
    vec4 q = mix(vec4(p.xyw, c.r), vec4(c.r, p.yzx), step(p.x, c.r));

    float d = q.x - min(q.w, q.y);
    float e = 1.0e-10;
    return vec3(abs(q.z + (q.w - q.y) / (6.0 * d + e)), d / (q.x + e), q.x);
 }
 
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 uniform float gain;
 void main()
 {
     vec4 val = texture2D(uInputTex, vTexCoord);
     gl_FragColor = vec4(rgb2hsv(val), 1.0);
 });
END_OGLES_GPGPU
// clang-format on
