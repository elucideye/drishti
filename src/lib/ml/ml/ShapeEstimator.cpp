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
#include "geometry/Ellipse.h"
#include "core/Shape.h"
#include "core/Logger.h"

#include <deque>

#if MOVE_RTE_STUFF

#include <dlib/image_transforms/assign_image.h>
#include <dlib/statistics/statistics.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/vectorstream.h>
#include <dlib/serialize.h>

#include "ml/shape_predictor.h"

//#define _SHAPE_PREDICTOR dlib

// This type uses the regression trees from dlib modified to support
// shape space model regression to reduce memory requirements.  In
// addition a few other options have been added such as normalized pixel
// differences and the line indexed features described in RCPC publication.
#define _SHAPE_PREDICTOR drishti::ml::shape_predictor

BOOST_CLASS_IMPLEMENTATION(_SHAPE_PREDICTOR, boost::serialization::object_class_info);
BOOST_CLASS_TRACKING(_SHAPE_PREDICTOR, boost::serialization::track_always);


#endif // MOVE_RTE_STUFF

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
