/*!
  @file   triangle.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of a separable ogles_gpgpu simple triangle filter shader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

/**
 * GPGPU gaussian smoothing processor (two-pass).
 */
#ifndef __drishti_acf_gpu_triangle_h__
#define __drishti_acf_gpu_triangle_h__

#include "drishti/acf/drishti_acf.h"
#include "ogles_gpgpu/common/common_includes.h"
#include "ogles_gpgpu/common/proc/base/multipassproc.h"
#include "drishti/acf/gpu/multipass/triangle_pass.h"

BEGIN_OGLES_GPGPU

class TriangleProc : public MultiPassProc
{
public:
    TriangleProc(int radius, bool doNorm = false, float normConst = 0.005f);
    /**
     * Return the processors name.
     */
    virtual const char* getProcName()
    {
        return "TriangleProc";
    }
};

END_OGLES_GPGPU

#endif
