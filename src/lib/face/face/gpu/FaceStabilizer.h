/*!
  @file   face/gpu/FaceStabilizer.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief Simple interface to stabilize the eyes (similarity transformation).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef FACE_STABILIZER_H
#define FACE_STABILIZER_H

#include "face/Face.h"
#include "eye/Eye.h"
#include "eye/gpu/EyeWarp.h" // utility

#include <opencv2/core.hpp>

//BEGIN_OGLES_GPGPU

class FaceStabilizer
{
public:
    FaceStabilizer(const cv::Size &sizeOut);
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
    std::array<EyeWarp, 2> renderEyes(const drishti::face::FaceModel &face, const cv::Size &sizeIn) const;
    std::array<EyeWarp, 2> renderEyes(const std::array<cv::Point2f, 2> &eyes, const cv::Size &sizeIn) const;

protected:

    bool m_autoScaling = false;

    // Per eye geometry:
    float m_aspectRatio = 3.0 / 4.0;
    int m_maxEyeWidth = 400;

    // Texture size:
    cv::Size m_sizeOut;
};

//END_OGLES_GPGPU

#endif // FACE_STABILIZER_H
