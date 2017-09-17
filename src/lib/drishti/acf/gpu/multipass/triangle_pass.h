/*! -*-c++-*-
  @file   triangle_pass.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of a single pass ogles_gpgpu triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

/**
 * GPGPU gaussian smoothing processor.
 */
#ifndef __drishti_acf_gpu_multipass_triangle_pass_h__
#define __drishti_acf_gpu_multipass_triangle_pass_h__

#include "drishti/core/drishti_core.h"
#include "ogles_gpgpu/common/common_includes.h"
#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

/**
 * This filter applies gaussian smoothing to an input image.
 */
class TriangleProcPass : public FilterProcBase
{
public:
    /**
     * Construct as render pass <pass> (1 or 2).
     */
    TriangleProcPass(int pass, float radius, bool doNorm = false, float normConst = 0.005f)
        : FilterProcBase()
        , doNorm(doNorm)
        , renderPass(pass)
        , pxDx(0.0f)
        , pxDy(0.0f)
        , normConst(normConst)
    {
        setRadius(radius);
        assert(renderPass == 1 || renderPass == 2);
    }

    void setRadius(int newValue);

    /**
     * Return the processors name.
     */
    virtual const char* getProcName()
    {
        return "TriangleProcPass";
    }

    virtual void filterShaderSetup(const char* vShaderSrc, const char* fShaderSrc, GLenum target);
    virtual void setUniforms();
    virtual void getUniforms();
    virtual const char* getFragmentShaderSource();
    virtual const char* getVertexShaderSource();

private:
    bool doNorm = false;
    int renderPass; // render pass number. must be 1 or 2

    float pxDx; // pixel delta value for texture access
    float pxDy; // pixel delta value for texture access

    float normConst = 0.005;

    float _blurRadiusInPixels = 0.0; // start 0 (uninitialized)

    GLint shParamUTexelWidthOffset;
    GLint shParamUTexelHeightOffset;

    std::string vshaderTriangleSrc;
    std::string fshaderTriangleSrc;
};

END_OGLES_GPGPU
#endif
