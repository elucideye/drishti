/*! -*-c++-*-
  @file   finder/LineDrawing.hpp
  @author David Hirvonen
  @brief Simple 2D line drawing representation for graphic overlay.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_gpu_LineDrawing_hpp__
#define __drishti_hci_gpu_LineDrawing_hpp__

#include <opencv2/core/core.hpp>
#include <vector>

#include "ogles_gpgpu/common/common_includes.h"

BEGIN_OGLES_GPGPU

struct LineDrawing
{
    bool strip = true;
    cv::Rect roi;
    cv::Vec3f color = { 0.0, 1.0, 0.0 };
    std::vector<std::vector<cv::Point2f>> contours;
    std::vector<int> index;
};

END_OGLES_GPGPU

#endif // __drishti_hci_gpu_LineDrawing_hpp__
