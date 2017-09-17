/*! -*-c++-*-
  @file   ObjectDetector.cpp
  @author David Hirvonen
  @brief  Internal ObjectDetector abstract API implementation file (high level routines).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/make_unique.h"

//#include "drishti/ml/ObjectDetectorCV.h"

#include "ObjectDetectorCV.h"

#include <opencv2/objdetect.hpp>

DRISHTI_ML_NAMESPACE_BEGIN

ObjectDetectorCV::ObjectDetectorCV(const std::string& filename)
{
    m_classifier = drishti::core::make_unique<cv::CascadeClassifier>(filename);
}

ObjectDetectorCV::~ObjectDetectorCV()
{
}

int ObjectDetectorCV::operator()(const cv::Mat& image, std::vector<cv::Rect>& objects, std::vector<double>* scores)
{
    m_classifier->detectMultiScale(image, objects, m_scaleStep, m_minNeighbors, 0, m_minSize, m_maxSize);
    return 0;
}

int ObjectDetectorCV::operator()(const MatP& image, std::vector<cv::Rect>& objects, std::vector<double>* scores)
{
    assert(false);
    return 0;
}

cv::Size ObjectDetectorCV::getWindowSize() const
{
    return cv::Size();
}

DRISHTI_ML_NAMESPACE_END
