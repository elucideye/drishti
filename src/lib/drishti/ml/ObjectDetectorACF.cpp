/*! -*-c++-*-
  @file   ObjectDetectorACF.cpp
  @author David Hirvonen
  @brief  Internal ObjectDetectorACF abstract API implementation file (high level routines).

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/ObjectDetectorACF.h"
#include "drishti/core/make_unique.h"

#include <acf/ACF.h>

DRISHTI_ML_NAMESPACE_BEGIN

ObjectDetectorACF::ObjectDetectorACF() = default;

ObjectDetectorACF::ObjectDetectorACF(const std::string& filename)
{
    m_impl = drishti::core::make_unique<acf::Detector>(filename);
}

ObjectDetectorACF::ObjectDetectorACF(std::istream& is, const std::string& hint)
{
    m_impl = drishti::core::make_unique<acf::Detector>(is, hint);
}

ObjectDetectorACF::~ObjectDetectorACF() = default;

int ObjectDetectorACF::operator()(const cv::Mat& image, std::vector<cv::Rect>& objects, std::vector<double>* scores)
{
    return (*m_impl)(image, objects, scores);
}

int ObjectDetectorACF::operator()(const MatP& image, std::vector<cv::Rect>& objects, std::vector<double>* scores)
{
    return (*m_impl)(image, objects, scores);
}

bool ObjectDetectorACF::good() const
{
    return m_impl->good();
}

ObjectDetectorACF::operator bool() const
{
    return m_impl->operator bool();
}

void ObjectDetectorACF::setDoNonMaximaSuppression(bool flag)
{
    m_impl->setDoNonMaximaSuppression(flag);
}

bool ObjectDetectorACF::getDoNonMaximaSuppression() const
{
    return m_impl->getDoNonMaximaSuppression();
}

cv::Size ObjectDetectorACF::getWindowSize() const
{
    return m_impl->getWindowSize();
}

DRISHTI_ML_NAMESPACE_END
