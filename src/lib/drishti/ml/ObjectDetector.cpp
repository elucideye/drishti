/*!
  @file   ObjectDetector.cpp
  @author David Hirvonen
  @brief  Internal ObjectDetector abstract API implementation file (high level routines).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <stdio.h>

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/ObjectDetector.h"

DRISHTI_ML_NAMESPACE_BEGIN

void ObjectDetector::prune(std::vector<cv::Rect> &objects, std::vector<double> &scores)
{
    CV_Assert(objects.size() == scores.size());

    // Assum sorted detections:
    if(objects.size() > 1)
    {
        int cutoff = 1;
        for(int i = 1; i < std::min(m_maxDetectionCount, objects.size()); i++)
        {
            cutoff = i;
            if(scores[i] < (scores[0] * m_detectionScorePruneRatio))
            {
                break;
            }
        }
        objects.erase(objects.begin() + cutoff, objects.end());
        scores.erase(scores.begin() + cutoff, scores.end());
    }
}

DRISHTI_ML_NAMESPACE_END
