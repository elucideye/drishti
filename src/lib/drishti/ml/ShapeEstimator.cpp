/*! -*-c++-*-
  @file   ShapeEstimator.cpp
  @author David Hirvonen
  @brief  Internal declaration of shape estimator API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains implementations for an abstract ShapeEstimator API class for 2D point
  based model estimation and a RegressionTreeEnsembleShapeEstimator variant.

*/

#include <stdio.h>

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/core/Shape.h"
#include "drishti/core/Logger.h"

#include <deque>

DRISHTI_ML_NAMESPACE_BEGIN

ShapeEstimator::~ShapeEstimator() = default;

void ShapeEstimator::setDoPreview(bool flag)
{
}

int ShapeEstimator::operator()(const cv::Mat& image, const cv::Rect& roi, Point2fVec& points, BoolVec& mask) const
{
    cv::Rect validRoi = roi & cv::Rect({ 0, 0 }, image.size());
    cv::Mat crop = image(validRoi);
    if (validRoi != roi)
    {
        cv::Mat padded(roi.size(), crop.type(), cv::Scalar::all(0));
        crop.copyTo(padded(validRoi - roi.tl()));
        cv::swap(crop, padded);
    }

    int n = (*this)(image(validRoi), points, mask);
    for (auto& p : points)
    {
        p.x += validRoi.x;
        p.y += validRoi.y;
    }
    return n;
}

DRISHTI_ML_NAMESPACE_END
