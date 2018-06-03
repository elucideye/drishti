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
    const char* getProcName() override
    {
        return "TriangleStripWarp";
    }

    const char* getVertexShaderSource() override
    {
        return vshaderDefault;
    }
    const char* getFragmentShaderSource() override
    {
        return fshaderTriangleSrc;
    }

    // We will override texture and vertex coords:
    void filterRenderSetCoords() override;
    void filterRenderDraw() override;

    PointSet2f m_pixels;
    PointSet2f m_texels;

    static const char* fshaderTriangleSrc;
};

END_OGLES_GPGPU

#endif // __drishti_eye_gpu_TriangleStripWarp_h__
