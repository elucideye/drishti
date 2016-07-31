/*!
  @file   finder/gpu/MultiTransformProc.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief Simple interface to apply parametric transformations to multiple image ROI's.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef MULTI_TRANSFORM_PROC_H
#define MULTI_TRANSFORM_PROC_H

#include "ogles_gpgpu/common/proc/transform.h"

BEGIN_OGLES_GPGPU

struct MappedTextureRegion
{
    Rect2d roi;
    Mat44f H; // column major format for opengl
};

class MultiTransformProc : public ogles_gpgpu::TransformProc
{
public:

    MultiTransformProc() {}
    virtual ~MultiTransformProc() {}

    void renderRegion(const Rect2d &dstRoiPix, const Mat44f &Heye);
    virtual void filterRenderDraw();

    void addCrop(const MappedTextureRegion &crop)
    {
        m_crops.push_back(crop);
    }

protected:

    std::vector<MappedTextureRegion> m_crops;
};

END_OGLES_GPGPU

#endif // MULTI_TRANSFORM_PROC_H
