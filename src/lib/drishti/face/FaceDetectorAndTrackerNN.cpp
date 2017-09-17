/*! -*-c++-*-
  @file   FaceDetectorAndTrackerNN.h
  @author David Hirvonen
  @brief  A nearest neighbor (noop) face tracking variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorAndTrackerNN.h"

DRISHTI_FACE_NAMESPACE_BEGIN

// =============================

TrackerNN::TrackerNN()
{
}

TrackerNN::~TrackerNN()
{
}

std::vector<cv::Point2f> TrackerNN::getFeatures() const
{
    std::vector<cv::Point2f> features;
    return features;
}

void TrackerNN::initialize(const cv::Mat1b& image, const FaceModel& face)
{
    m_face = face;
}

bool TrackerNN::update(const cv::Mat1b& image, FaceModel& face)
{
    face = m_face;
    return true;
}

DRISHTI_FACE_NAMESPACE_END
