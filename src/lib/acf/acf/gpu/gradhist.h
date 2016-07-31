/*!
  @file   gradhist.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of an ogles_gpgpu shader for computing gradient orientation histograms.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_GRADHIST_H
#define OGLES_GPGPU_GRADHIST_H

#include "acf/drishti_acf.h"
#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

//======= GradHist =========

class GradHistProc : public ::ogles_gpgpu::FilterProcBase
{
public:
    GradHistProc(int nOrientations, int base=0, float strength = 1.f)
        : nOrientations(nOrientations)
        , strength(strength)
    {
        setBase(base);
    }
    virtual const char *getProcName()
    {
        return "GradHistProc";
    }
    void setBase(int b)
    {
        for(int i = 0; i < 4; i++)
        {
            index[i] = b + i;
        }
    }
private:
    virtual const char *getFragmentShaderSource()
    {
        return fshaderGradHistSrcN;
    }
    virtual void getUniforms()
    {
        FilterProcBase::getUniforms();
        shParamStrength = shader->getParam(UNIF, "strength");
        shParamUIndex = shader->getParam(UNIF, "index");
        shParamUNOrientations = shader->getParam(UNIF, "nOrientations");
    }
    virtual void setUniforms()
    {
        FilterProcBase::setUniforms();
        glUniform1f(shParamStrength, strength);
        glUniform4i(shParamUIndex, index[0], index[1], index[2], index[3]);
        glUniform1f(shParamUNOrientations, nOrientations);
    }

    float nOrientations = 6;
    float strength = 1.0;
    int index[4] = { 0, 1, 2, 3 };

    GLint shParamStrength;
    GLint shParamUIndex;
    GLint shParamUNOrientations;

    static const char *fshaderGradHistSrcN;   // fragment shader source
};

END_OGLES_GPGPU

#endif // OGLES_GPGPU_GRADHIST_H
