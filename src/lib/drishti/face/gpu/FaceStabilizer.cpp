/*! -*-c++-*-
  @file   face/gpu/FaceStabilizer.cpp
  @author David Hirvonen
  @brief Simple interface to stabilize the eyes (similarity transformation).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/geometry/motion.h" // for transformation::
#include "drishti/core/Shape.h"

DRISHTI_FACE_NAMESPACE_BEGIN

FaceStabilizer::FaceStabilizer(const cv::Size& sizeOut)
    : m_sizeOut(sizeOut)
{
}

cv::Matx33f FaceStabilizer::stabilize(const drishti::face::FaceModel& face, const cv::Size& sizeOut, float span)
{
    using PointPair = std::array<cv::Point2f, 2>;

    PointPair eyeCenters{ { face.eyeFullR->irisEllipse.center, face.eyeFullL->irisEllipse.center } };
    if (std::min(face.eyeFullR->openness(), face.eyeFullL->openness()) < 0.1f)
    {
        eyeCenters = { { core::centroid(face.eyeRight), core::centroid(face.eyeLeft) } };
    }

    // clang-format off
    const PointPair screenCenters
    {{
        transformation::denormalize(cv::Point2f(0.5-span*0.5, 0.5), sizeOut),
        transformation::denormalize(cv::Point2f(0.5+span*0.5, 0.5), sizeOut)
    }};
    // clang-format on
    return transformation::estimateSimilarity(eyeCenters, screenCenters);
}

std::array<eye::EyeWarp, 2>
FaceStabilizer::renderEyes(const drishti::face::FaceModel& face, const cv::Size& sizeIn) const
{
    using PointPair = std::array<cv::Point2f, 2>;

    std::array<eye::EyeWarp, 2> eyes;
    if (face.eyeFullR.has && face.eyeFullL.has)
    {
        const PointPair eyeCenters{ { face.eyeFullR->irisEllipse.center, face.eyeFullL->irisEllipse.center } };
        eyes = renderEyes(eyeCenters, sizeIn);
        eyes[0].eye = face.eyeFullR;
        eyes[1].eye = face.eyeFullL;
    }

    return eyes;
}

std::array<eye::EyeWarp, 2>
FaceStabilizer::renderEyes(const std::array<cv::Point2f, 2>& eyeCenters, const cv::Size& sizeIn) const
{
    using PointPair = std::array<cv::Point2f, 2>;

    const cv::Size screenSize(m_sizeOut.width, m_sizeOut.height);
    const cv::Point screenCenter(screenSize.width / 2, screenSize.height / 2);

    // SCREEN: map pixels to normalized texture [-1.0 ... +1.0]
    cv::Matx33f No = transformation::normalize(screenSize);

    // Map pixel from normalized texture to input pixels
    //cv::Matx33f Ni = transformation::denormalize(sizeIn);

    int eyeWidth = screenSize.width / 2;
    std::array<cv::Rect, 2> eyeRois;

    if (m_autoScaling) // auto
    {
        const cv::Rect eyeRoi(0, 0, screenSize.width / 2, screenSize.height);
        eyeRois = { { eyeRoi, eyeRoi + cv::Point(eyeRoi.width, 0) } };
    }
    else
    {
        eyeWidth = std::min(screenSize.width / 2, m_maxEyeWidth);
        const cv::Point eyeOffset(eyeWidth / 2, 0);
        const cv::Rect eyeRoi(screenCenter.x - eyeOffset.x, 0, eyeWidth, int(m_aspectRatio * eyeWidth + 0.5f));
        eyeRois = { { eyeRoi - eyeOffset, eyeRoi + eyeOffset } };
    }

    const PointPair screenCenters{ { transformation::center(eyeRois[0]), transformation::center(eyeRois[1]) } };
    const float eyeScaleInScreen = cv::norm(screenCenters[0] - screenCenters[1]);
    cv::Matx33f H = transformation::estimateSimilarity(eyeCenters, screenCenters);

    std::array<eye::EyeWarp, 2> cropInfo;
    for (int i = 0; i < 2; i++)
    {
        // Adjust scaling
        const float scale = 2.f * float(eyeWidth) / eyeScaleInScreen;
        const cv::Matx33f S = transformation::scale(scale, scale, screenCenters[i]);
        const cv::Matx33f Heye = No * S * H;
        cropInfo[i] = { eyeRois[i], Heye }; // return this for use later
    }

    return cropInfo;
}

DRISHTI_FACE_NAMESPACE_END
