/*! -*-c++-*-
  @file   binomial.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of an ogles_gpgpu binomial filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_binomial_h__
#define __drishti_graphics_binomial_h__

#include "ogles_gpgpu/common/proc/filter3x3.h"

BEGIN_OGLES_GPGPU

//======= Binomial =============

class BinomialProc : public ogles_gpgpu::Filter3x3Proc
{
public:
    BinomialProc() = default;
    const char* getProcName() override
    {
        return "BinomialProc";
    }

private:
    const char* getFragmentShaderSource() override
    {
        return fshaderBinomialSrc;
    }
    void getUniforms() override
    {
        Filter3x3Proc::getUniforms();
        shParamUInputTex = shader->getParam(UNIF, "inputImageTexture");
    }
    void setUniforms() override {}
    static const char* fshaderBinomialSrc; // fragment shader source
};

END_OGLES_GPGPU

#endif //  __drishti_graphics_binomial_h__
