/*! -*-c++-*-
  @file fill.h
  @brief Declarations for GPUImage fill geometry computations for ogles_gpgpu

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Replicates the GPUImage fill geometries for ogles_gpgpu

*/

#ifndef __facefilter_renderer_fill_h__
#define __facefilter_renderer_fill_h__

#include <opencv2/core.hpp>

enum FillMode
{
    kFillModeStretch,
    kFillModePreserveAspectRatio,
    kFillModePreserveAspectRatioAndFill
};

struct RectTransform
{
    float x, y, scaleX, scaleY;
};

RectTransform
recalculateViewGeometry(const cv::Size& inputImageSize, const cv::Size& currentViewSize, FillMode fillMode);

#endif // __facefilter_renderer_fill_h__
