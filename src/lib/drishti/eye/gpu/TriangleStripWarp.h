/*! -*-c++-*-
  @file   TriangleStripWarp.h
  @author David Hirvonen
  @brief  Declaration of utility structure to perform arbitrary warps via GL_TRIANGL_STRIPS

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_eye_gpu_TriangleStripWarp_h__
#define __drishti_eye_gpu_TriangleStripWarp_h__

#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/common_includes.h"

#include <opencv2/core.hpp>

BEGIN_OGLES_GPGPU

class TriangleStripWarp : public ogles_gpgpu::FilterProcBase
{
public:
    using PointSet2f = std::vector<cv::Point2f>;

    TriangleStripWarp();
    virtual const char* getProcName()
    {
        return "TriangleStripWarp";
    }

    virtual const char* getVertexShaderSource()
    {
        return vshaderDefault;
    }
    virtual const char* getFragmentShaderSource()
    {
        return fshaderTriangleSrc;
    }

    // We will override texture and vertex coords:
    virtual void filterRenderSetCoords();
    virtual void filterRenderDraw();

    PointSet2f m_pixels;
    PointSet2f m_texels;

    static const char* fshaderTriangleSrc;
};

END_OGLES_GPGPU

#endif // __drishti_eye_gpu_TriangleStripWarp_h__
