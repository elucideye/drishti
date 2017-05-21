//
// ogles_gpgpu project - GPGPU for mobile devices and embedded systems using OpenGL ES 2.0
//
// Author: Markus Konrad <post@mkonrad.net>, Winter 2014/2015 (http://www.mkonrad.net)
//         David Hirvonen
//
// See LICENSE file in project repository root for the license.
//

/**
 * GPGPU grad processor.
 */
#ifndef OGLES_GPGPU_COMMON_PROC_NMS
#define OGLES_GPGPU_COMMON_PROC_NMS

#include "ogles_gpgpu/common/proc/filter3x3.h"

BEGIN_OGLES_GPGPU

/**
 * GPGPU non max suppression, gradient magnitude and orientation 
 */
class Nms2Proc : public Filter3x3Proc
{
public:
    /**
     * Constructor.
     */
    Nms2Proc();

    /**
     * Return the processors name.
     */
    virtual const char* getProcName()
    {
        return "Nms2Proc";
    }

    void swizzle(int channelIn, int channelOut = -1);

private:
    /**
     * Get the fragment shader source.
     */
    virtual const char* getFragmentShaderSource()
    {
        return fshaderNmsSwizzleSrc.empty() ? fshaderNmsSrc : fshaderNmsSwizzleSrc.c_str();
    }

    /**
     * Get uniform indices.
     */
    virtual void getUniforms();

    /**
     * Set uniforms.
     */
    virtual void setUniforms();

    static const char* fshaderNmsSrc; // fragment shader source

    std::string fshaderNmsSwizzleSrc;
};

END_OGLES_GPGPU

#endif
