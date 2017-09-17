/*! -*-c++-*-
  @file   ShapeEstimator.h
  @author David Hirvonen
  @brief  Internal declaration of shape estimator API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains declarations for an abstract ShapeEstimator API class for 2D point
  based model estimation and a RegressionTreeEnsembleShapeEstimator variant.

*/

#ifndef __drishti_ml_ShapeEstimator_h__
#define __drishti_ml_ShapeEstimator_h__

#include "drishti/ml/drishti_ml.h"
#include "drishti/core/Logger.h"

#include <opencv2/core.hpp>

#include <memory>
#include <vector>

DRISHTI_ML_NAMESPACE_BEGIN

class ShapeEstimator
{
public:
    typedef std::vector<bool> BoolVec;
    typedef std::vector<cv::Point2f> Point2fVec;

    virtual ~ShapeEstimator();

    virtual void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        m_streamLogger = logger;
    }

    virtual int operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const = 0;
    virtual int operator()(const cv::Mat& crop, Point2fVec& points, BoolVec& mask) const = 0;
    virtual int operator()(const cv::Mat& image, const cv::Rect& roi, Point2fVec& points, BoolVec& mask) const;
    virtual std::vector<cv::Point2f> getMeanShape() const
    {
        return std::vector<cv::Point2f>();
    }
    virtual void setDoPreview(bool flag);
    virtual bool isPCA() const
    {
        return false;
    }
    virtual void setStagesHint(int stages){};
    virtual int getStagesHint() const
    {
        return 0;
    }

    virtual void dump(std::vector<float>& params, bool pca = false) {}

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version) {}

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

DRISHTI_ML_NAMESPACE_END

#endif
