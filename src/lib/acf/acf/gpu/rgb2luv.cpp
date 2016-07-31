/*!
  @file   rgb2luv.cpp
  @author David Hirvonen (C++ implementation) <dhirvonen elucideye com>
  @brief Implementation of an ogles_gpgpu shader for RGB to LUV colorspace conversions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "rgb2luv.h"

BEGIN_OGLES_GPGPU

// RGB to LUV with built in shift to [0,1] range
// see: https://github.com/pdollar/toolbox/blob/master/channels/rgbConvert.m
// L=L/270
// u=(u+88)/270
// v=(v+134)/270
//luv.x = (luv.x/270.0);
//luv.y = ((luv.y+88.0)/270.0);
//luv.z = ((luv.z+134.0)/270.0);

// *INDENT-OFF*
const char * Rgb2LuvProc::fshaderRgb2LuvSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
  precision highp float;
#endif

  varying vec2 vTexCoord;
  uniform sampler2D uInputTex;

  const mat3 RGBtoXYZ = mat3(0.430574, 0.341550, 0.178325, 0.222015, 0.706655, 0.071330, 0.020183, 0.129553, 0.93918);

  const float y0 = 0.00885645167; //pow(6.0/29.0, 3.0)
  const float a = 903.296296296;  //pow(29.0/3.0, 3.0);
  const float un = 0.197833;
  const float vn = 0.468331;
  const float maxi = 0.0037037037;  // 1.0/270.0;
  const float minu = -88.0 * maxi;
  const float minv = -134.0 * maxi;
  const vec3 k = vec3(1.0, 15.0, 3.0);
  const vec3 RGBtoGray = vec3(0.2125, 0.7154, 0.0721);

  void main()
  {
      vec3 rgb = texture2D(uInputTex, vTexCoord).rgb;
      vec3 xyz = (rgb * RGBtoXYZ);
      float z = 1.0/(dot(xyz,k) + 1e-35);

      vec3 luv;
      float y = xyz.y; // clamp(xyz.y, 0.0, 1.0);
      float l = ((y > y0) ? ((116.0 * pow(y, 0.3333333333)) - 16.0) : (y*a)) * maxi;
      luv.x = l;
      luv.y = l * ((52.0 * xyz.x * z) - (13.0*un)) - minu;
      luv.z = l * ((117.0 * xyz.y * z) - (13.0*vn)) - minv;

      gl_FragColor = vec4( vec3(luv.xyz), dot(rgb, RGBtoGray) );
      //gl_FragColor = vec4(0.0, 0.5, 1.0, 1.0);
  }
);
// *INDENT-ON*

END_OGLES_GPGPU
