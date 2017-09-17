/*! -*-c++-*-
  @file   ObjectDetectorCV.h
  @author David Hirvonen
  @brief  Internal ObjectDetectorCV abstract API declaration file.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_ml_ObjectDetectorCV_h__
#define __drishti_ml_ObjectDetectorCV_h__

#include "drishti/ml/ObjectDetector.h"

#include <opencv2/core.hpp>

#include <vector>

// clang-format off
namespace cv { class CascadeClassifier; }
// clang-format on

DRISHTI_ML_NAMESPACE_BEGIN

class ObjectDetectorCV : public ObjectDetector
{
public:
    ObjectDetectorCV(const std::string& filename);
    ~ObjectDetectorCV();

    virtual int operator()(const cv::Mat& image, std::vector<cv::Rect>& objects, std::vector<double>* scores = 0);
    virtual int operator()(const MatP& image, std::vector<cv::Rect>& objects, std::vector<double>* scores = 0);
    virtual cv::Size getWindowSize() const;

    void setMinNeighbors(int value) { m_minNeighbors = value; }
    void setScaleStep(float value) { m_scaleStep = value; }
    void setMinSize(const cv::Size& size) { m_minSize = size; }
    void setMaxSize(const cv::Size& size) { m_maxSize = size; }

protected:
    std::unique_ptr<cv::CascadeClassifier> m_classifier;

    int m_minNeighbors = 1;
    float m_scaleStep = 1.1f;
    cv::Size m_minSize; // default to min template
    cv::Size m_maxSize; // default to full image
};

DRISHTI_ML_NAMESPACE_END

#endif
