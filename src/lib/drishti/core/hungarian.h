/*!
  @file   hungarian.h
  @brief  Adaptation of hungarian assignment algorithm from OpenCV:

  Lineage : opencv/sources/opencv/modules/shape/src/sc_dis.cpp:SCDMatcher::hungarian

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This file is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h" // namespace definition

#include <opencv2/core/core.hpp>

#include <iostream>

#ifndef __drishti_core_hungarian_h__
#define __drishti_core_hungarian_h__

DRISHTI_CORE_NAMESPACE_BEGIN

using DMatchVec = std::vector<cv::DMatch>;
using FloatVec = std::vector<float>;
using IntVec = std::vector<int>;

double hungarian(const cv::Mat &costMatrix, DMatchVec &outMatches, IntVec &inliers1, IntVec &inliers2, int sizeScd1, int sizeScd2);

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_hungarian_h__
