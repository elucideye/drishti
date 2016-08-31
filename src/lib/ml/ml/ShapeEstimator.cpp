/*!
  @file   ShapeEstimator.cpp
  @author David Hirvonen
  @brief  Internal declaration of shape estimator API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains implementations for an abstract ShapeEstimator API class for 2D point
  based model estimation and a RegressionTreeEnsembleShapeEstimator variant.

*/

#include <stdio.h>

#include "ml/drishti_ml.h"
#include "ml/ShapeEstimator.h"
#include "geometry/Primitives.h"
#include "core/Shape.h"
#include "core/Logger.h"

#include <deque>

_DRISHTI_ML_BEGIN

void ShapeEstimator::setDoPreview(bool flag)
{
}

int ShapeEstimator::operator()(const cv::Mat &image, const cv::Rect &roi, Point2fVec &points, BoolVec &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6,1,m_streamLogger);

    cv::Rect validRoi = roi & cv::Rect({0,0}, image.size());
    cv::Mat crop = image(validRoi);
    if(validRoi != roi)
    {
        cv::Mat padded(roi.size(), crop.type(), cv::Scalar::all(0));
        crop.copyTo(padded(validRoi - roi.tl()));
        cv::swap(crop, padded);
    }

    int n = (*this)(image(validRoi), points, mask);
    for(auto &p : points)
    {
        p.x += validRoi.x;
        p.y += validRoi.y;
    }
    return n;
}

_DRISHTI_ML_END
