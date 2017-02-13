/*!
  @file   rgb2luv.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of an ogles_gpgpu shader for RGB to LUV colorspace conversions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_rgb2luv_h__
#define __drishti_graphics_rgb2luv_h__

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

BEGIN_OGLES_GPGPU

// ######## RGB 2 LUV ############

class Rgb2LuvProc : public FilterProcBase
{
public:
    Rgb2LuvProc() {}
    virtual const char *getProcName()
    {
        return "Rgb2LuvProc";
    }
private:
    virtual const char *getFragmentShaderSource()
    {
        return fshaderRgb2LuvSrc;
    }
    virtual void getUniforms() {}
    static const char *fshaderRgb2LuvSrc;   // fragment shader source
};

END_OGLES_GPGPU

#endif // __drishti_graphics_rgb2luv_h__
