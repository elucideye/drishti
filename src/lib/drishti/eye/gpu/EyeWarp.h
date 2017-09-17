/*! -*-c++-*-
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

DRISHTI_EYE_NAMESPACE_BEGIN

struct EyeWarp
{
    EyeWarp() {}
    EyeWarp(const cv::Rect2f& roi, const cv::Matx33f& H)
        : roi(roi)
        , H(H)
    {
    }
    EyeWarp(const cv::Rect2f& roi, const cv::Matx33f& H, const DRISHTI_EYE::EyeModel& eye)
        : roi(roi)
        , H(H)
        , eye(eye)
    {
    }
    cv::Rect2f roi;
    cv::Matx33f H;
    DRISHTI_EYE::EyeModel eye;

    const std::vector<std::vector<cv::Point2f>>& getContours(bool doPupil = false) const
    {
        return m_contours[int(doPupil)];
    }

    void setContours(const std::vector<std::vector<cv::Point2f>>& contour, bool doPupil = false)
    {
        m_contours[int(doPupil)] = contour;
    }

    const std::vector<std::vector<cv::Point2f>>& getContours(bool doPupil = false)
    {
        auto& contours = m_contours[int(doPupil)];
        if (!contours.size())
        {
            contours = eye.getContours(doPupil);
        }
        return contours;
    }

    // For efficiency, we add a model contour cache:
    std::vector<std::vector<cv::Point2f>> m_contours[2];
};

DRISHTI_EYE_NAMESPACE_END

#endif // __drishti_eye_gpu_EyeWarp_h__
