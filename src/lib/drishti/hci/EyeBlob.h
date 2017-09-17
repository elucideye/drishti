/*! -*-c++-*-
  @file   EyeBlob.h
  @author David Hirvonen
  @brief  Internal declaration for a utility feature point extraction.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_EyeBlob_h__
#define __drishti_hci_EyeBlob_h__

#include "drishti/hci/Scene.hpp"
#include "drishti/eye/gpu/EyeWarp.h"

DRISHTI_HCI_NAMESPACE_BEGIN

struct EyeBlobJob
{
    using FeaturePoints = std::vector<FeaturePoint>;

    EyeBlobJob(const cv::Size& size, const std::array<drishti::eye::EyeWarp, 2>& eyeWarps);
    FeaturePoints getValidEyePoints(const FeaturePoints& points, const drishti::eye::EyeWarp& eyeWarp, const cv::Size& size);
    void run();

    cv::Mat4b filtered;
    cv::Mat1b alpha;
    const std::array<drishti::eye::EyeWarp, 2>& eyeWarps;
    std::array<FeaturePoints, 2> eyePoints;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_EyeBlob_h__
