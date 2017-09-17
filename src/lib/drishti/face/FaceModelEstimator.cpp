/*! -*-c++-*-
  @file   FaceModelEstimator.cpp
  @author David Hirvonen
  @brief  Internal declaration of a class to estimate 3D models from 2D face landmarks.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceModelEstimator.h"

DRISHTI_FACE_NAMESPACE_BEGIN

FaceModelEstimator::FaceModelEstimator(const sensor::SensorModel& sensor)
    : m_sensor(sensor)
{
}

cv::Point3f FaceModelEstimator::getDepth(const DRISHTI_EYE::EyeModel& eyeR, const DRISHTI_EYE::EyeModel& eyeL)
{
    std::array<cv::Point2f, 2> eyes{ { eyeR.irisEllipse.center, eyeL.irisEllipse.center } };
    cv::Point3f P = m_sensor.intrinsic().getDepth(eyes, 0.064);
    return P;
}

cv::Point3f FaceModelEstimator::operator()(const face::FaceModel& face)
{
    return getDepth(face.eyeFullR, face.eyeFullL);
}

DRISHTI_FACE_NAMESPACE_END
