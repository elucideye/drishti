/*! -*-c++-*-
  @file swizzle.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of an ogles_gpgpu shader for common texture channel swizzles.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_swizzle_h__
#define __drishti_graphics_swizzle_h__

#include "ogles_gpgpu/common/proc/two.h"

BEGIN_OGLES_GPGPU

// ##### Simple single texture swizzling: #######

class SwizzleProc : public ogles_gpgpu::FilterProcBase
{
public:
    // Relative to input {RGBA}
    enum SwizzleKind
    {
        kSwizzleRGBA,
        kSwizzleBGRA,
        kSwizzleARGB,
        kSwizzleABGR,
        kSwizzleGRAB
    };

    SwizzleProc(SwizzleKind swizzleKind = kSwizzleBGRA)
        : swizzleKind(swizzleKind)
    {
    }
    virtual const char* getProcName()
    {
        return "SwizzleProc";
    }
    void setSwizzleType(SwizzleKind kind)
    {
        swizzleKind = kind;
    }
    virtual const char* getFragmentShaderSource()
    {
        switch (swizzleKind)
        {
            case kSwizzleRGBA:
                return fshaderRGBASrc;
            case kSwizzleBGRA:
                return fshaderBGRASrc;
            case kSwizzleARGB:
                return fshaderARGBSrc;
            case kSwizzleABGR:
                return fshaderABGRSrc;
            case kSwizzleGRAB:
                return fshaderGRABSrc;
            default:
                assert(false);
        }
    }

    static const char* fshaderRGBASrc;
    static const char* fshaderBGRASrc; // fragment shader source
    static const char* fshaderARGBSrc;
    static const char* fshaderABGRSrc;
    static const char* fshaderGRABSrc;
    SwizzleKind swizzleKind = kSwizzleBGRA;
};

END_OGLES_GPGPU

#endif // __drishti_graphics_swizzle_h__
