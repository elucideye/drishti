/*!
  @file   ObjectDetector.h
  @author David Hirvonen
  @brief  Internal ObjectDetector abstract API declaration file.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_ObjectDetector_h
#define drishtisdk_ObjectDetector_h

#include "ml/drishti_ml.h"
#include "acf/MatP.h"

#include <opencv2/core/core.hpp>

#include <vector>

_DRISHTI_ML_BEGIN

// Specify API
class ObjectDetector
{
public:

    // TODO: enforce a public non virtual API that calls a virtual detect method
    // and applies pruning criteria as needed.
    virtual int operator()(const cv::Mat &image, std::vector<cv::Rect> &objects, std::vector<double> *scores=0) = 0;
    virtual int operator()(const MatP &image, std::vector<cv::Rect> &objects, std::vector<double> *scores=0) = 0;
    virtual void setMaxDetectionCount(size_t maxCount)
    {
        m_maxDetectionCount = maxCount;
    }
    virtual void setDetectionScorePruneRatio(double ratio)
    {
        m_detectionScorePruneRatio = ratio;
    }
    virtual void prune(std::vector<cv::Rect> &objects, std::vector<double> &scores);
    virtual void setDoNonMaximaSuppression(bool flag)
    {
        m_doNms = flag;
    }
    virtual bool getDoNonMaximaSuppression() const
    {
        return m_doNms;
    }
protected:
    bool m_doNms = false;
    double m_detectionScorePruneRatio = 0.0;
    size_t m_maxDetectionCount = 10;
};

_DRISHTI_ML_END

#endif
