/*! -*-c++-*-
  @file   rgb2hsv.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for rgb2hsv.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_COMMON_RGB2HSV_PROC
#define OGLES_GPGPU_COMMON_RGB2HSV_PROC

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

class Rgb2HsvProc : public ogles_gpgpu::FilterProcBase
{
public:
    Rgb2HsvProc(float gain = 1.f)
        : gain(gain)
    {
    }
    virtual const char* getProcName()
    {
        return "Rgb2HsvProc";
    }

private:
    virtual const char* getFragmentShaderSource()
    {
        return fshaderRgb2HsvSrc;
    }
    virtual void getUniforms()
    {
        shParamUGain = shader->getParam(UNIF, "gain");
    }
    virtual void setUniforms()
    {
        glUniform1f(shParamUGain, gain);
    }
    static const char* fshaderRgb2HsvSrc; // fragment shader source
    float gain = 1.f;
    GLint shParamUGain;
};

END_OGLES_GPGPU

#endif
