/*!
  @file   gain.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for gain.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_GAIN_H
#define OGLES_GPGPU_GAIN_H

#include "drishti/acf/drishti_acf.h"
#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

// ======= NOOP ========

class NoopProc : public ogles_gpgpu::FilterProcBase
{
public:
    NoopProc(float gain=1.f) : gain(gain) {}
    virtual const char *getProcName()
    {
        return "NoopProc";
    }
private:
    virtual const char *getFragmentShaderSource()
    {
        return fshaderNoopSrc;
    }
    virtual void getUniforms()
    {
        shParamUGain = shader->getParam(UNIF, "gain");
    }
    virtual void setUniforms()
    {
        glUniform1f(shParamUGain, gain);
    }
    static const char *fshaderNoopSrc; // fragment shader source
    float gain = 1.f;
    GLint shParamUGain;
};

END_OGLES_GPGPU

#endif //  OGLES_GPGPU_GAIN_H
