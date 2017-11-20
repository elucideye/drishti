/*! -*-c++-*-
  @file   ObjectDetector.cpp
  @author David Hirvonen
  @brief  Internal ObjectDetector abstract API implementation file (high level routines).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <stdio.h>

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/ObjectDetector.h"

#include <iostream>

DRISHTI_ML_NAMESPACE_BEGIN

void ObjectDetector::setMaxDetectionCount(size_t maxCount)
{
    m_maxDetectionCount = maxCount;
}

void ObjectDetector::setDoNonMaximaSuppression(bool flag)
{
    m_doNms = flag;
}

bool ObjectDetector::getDoNonMaximaSuppression() const
{
    return m_doNms;
}

void ObjectDetector::setDetectionScorePruneRatio(double ratio)
{
    m_detectionScorePruneRatio = ratio;
}

void ObjectDetector::prune(std::vector<cv::Rect>& objects, std::vector<double>& scores)
{
    CV_Assert(objects.size() == scores.size());

    // Assum sorted detections:
    if (objects.size() > 1)
    {
        int cutoff = 1;
        for (int i = 1; i < std::min(m_maxDetectionCount, objects.size()); i++)
        {
            cutoff = i + 1;
            if (scores[i] < (scores[0] * m_detectionScorePruneRatio))
            {
                break;
            }
        }
        objects.erase(objects.begin() + cutoff, objects.end());
        scores.erase(scores.begin() + cutoff, scores.end());
    }
}

DRISHTI_ML_NAMESPACE_END
