/*! -*-c++-*-
  @file   saturation.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for saturation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_COMMON_SATURATION_PROC
#define OGLES_GPGPU_COMMON_SATURATION_PROC

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

class SaturationProc : public ogles_gpgpu::FilterProcBase
{
public:
    SaturationProc(float gain = 1.f)
        : gain(gain)
    {
    }
    const char* getProcName() override
    {
        return "SaturationProc";
    }

private:
    const char* getFragmentShaderSource() override
    {
        return fshaderSaturationSrc;
    }
    void getUniforms() override
    {
        shParamUGain = shader->getParam(UNIF, "gain");
    }
    void setUniforms() override
    {
        glUniform1f(shParamUGain, gain);
    }
    static const char* fshaderSaturationSrc; // fragment shader source
    float gain = 1.f;
    GLint shParamUGain{};
};

END_OGLES_GPGPU

#endif
