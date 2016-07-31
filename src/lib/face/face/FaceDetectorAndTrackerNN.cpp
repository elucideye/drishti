/*!
  @file   FaceDetectorAndTrackerNN.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  A nearest neighbor (noop) face tracking variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "face/FaceDetectorAndTrackerNN.h"

BEGIN_FACE_NAMESPACE

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


void TrackerNN::initialize(const cv::Mat1b &image, const FaceModel &face)
{
    m_face = face;
}

bool TrackerNN::update(const cv::Mat1b &image, FaceModel &face)
{
    face = m_face;
    return true;
}


END_FACE_NAMESPACE
