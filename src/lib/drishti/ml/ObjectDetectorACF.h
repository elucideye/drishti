/*! -*-c++-*-
  @file   ObjectDetectorACF.h
  @author David Hirvonen
  @brief  Internal ObjectDetectorACF abstract API declaration file.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_ml_ObjectDetectorACF_h__
#define __drishti_ml_ObjectDetectorACF_h__

#include "drishti/ml/ObjectDetector.h"

#include <acf/ACF.h>

#include <vector>

DRISHTI_ML_NAMESPACE_BEGIN

class ObjectDetectorACF : public ObjectDetector
{
public:
    
    ObjectDetectorACF();
    ObjectDetectorACF(const std::string& filename);
    ObjectDetectorACF(std::istream& is, const std::string& hint = {});
    virtual ~ObjectDetectorACF();
    
    bool good() const;
    explicit operator bool() const;
    
    virtual int operator()(const cv::Mat& image, std::vector<cv::Rect>& objects, std::vector<double>* scores = 0);
    virtual int operator()(const MatP& image, std::vector<cv::Rect>& objects, std::vector<double>* scores = 0);
    virtual cv::Size getWindowSize() const;
    virtual void setDoNonMaximaSuppression(bool flag);
    virtual bool getDoNonMaximaSuppression() const;
    
    acf::Detector* getDetector() const { return m_impl.get(); }

protected:
    
    std::unique_ptr<acf::Detector> m_impl;

};

DRISHTI_ML_NAMESPACE_END

#endif
