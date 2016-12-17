/*!
  @file   EyeWarp.h
  @author David Hirvonen
  @brief  Declaration of utility structure to specify a transformed eye model.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_eye_gpu_EyeWarp_h__
#define __drishti_eye_gpu_EyeWarp_h__ 1

#include "drishti/eye/Eye.h"

#include <opencv2/core/core.hpp>

struct EyeWarp
{
    EyeWarp() {}
    EyeWarp(const cv::Rect2f &roi, const cv::Matx33f &H) : roi(roi), H(H) {}
    EyeWarp(const cv::Rect2f &roi, const cv::Matx33f &H, const DRISHTI_EYE::EyeModel &eye) : roi(roi), H(H), eye(eye) {}
    cv::Rect2f roi;
    cv::Matx33f H;
    DRISHTI_EYE::EyeModel eye;
};

#endif // __drishti_eye_gpu_EyeWarp_h__ 
