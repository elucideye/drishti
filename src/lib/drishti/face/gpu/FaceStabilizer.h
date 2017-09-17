/*! -*-c++-*-
  @file   face/gpu/FaceStabilizer.h
  @author David Hirvonen
  @brief Simple interface to stabilize the eyes (similarity transformation).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_gpu_FaceStabilizer_h__
#define __drishti_face_gpu_FaceStabilizer_h__

#include "drishti/face/Face.h"
#include "drishti/eye/Eye.h"
#include "drishti/eye/gpu/EyeWarp.h" // utility

#include <opencv2/core.hpp>

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceStabilizer
{
public:
    FaceStabilizer(const cv::Size& sizeOut);
    void setEyeWidth(int width)
    {
        m_maxEyeWidth = width;
    }
    void setEyeAspectRatio(float ratio)
    {
        m_aspectRatio = ratio;
    }
    void setDoAutoScaling(bool flag)
    {
        m_autoScaling = flag;
    }

    static cv::Matx33f stabilize(const drishti::face::FaceModel& face, const cv::Size& sizeOut, float span);
    std::array<eye::EyeWarp, 2> renderEyes(const drishti::face::FaceModel& face, const cv::Size& sizeIn) const;
    std::array<eye::EyeWarp, 2> renderEyes(const std::array<cv::Point2f, 2>& eyes, const cv::Size& sizeIn) const;

protected:
    bool m_autoScaling = false;

    // Per eye geometry:
    float m_aspectRatio = 3.0 / 4.0;
    int m_maxEyeWidth = 400;

    // Texture size:
    cv::Size m_sizeOut;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_gpu_FaceStabilizer_h__
