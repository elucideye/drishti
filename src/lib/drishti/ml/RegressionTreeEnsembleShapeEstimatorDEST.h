/*! -*-c++-*-
  @file   RegressionTreeEnsembleShapeEstimatorDEST.h
  @author David Hirvonen
  @brief  Internal declaration of regression tree ensemble shape estimator variant.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_ml_RegressionTreeEnsembleShapeEstimatorDEST_h__
#define __drishti_ml_RegressionTreeEnsembleShapeEstimatorDEST_h__

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/ShapeEstimator.h"

#include <opencv2/core/core.hpp>

#include <memory>
#include <vector>

DRISHTI_ML_NAMESPACE_BEGIN

// Consider occlusion estimation
class RegressionTreeEnsembleShapeEstimatorDEST : public ShapeEstimator
{
public:
    class Impl;

    RegressionTreeEnsembleShapeEstimatorDEST();
    virtual ~RegressionTreeEnsembleShapeEstimatorDEST();
    RegressionTreeEnsembleShapeEstimatorDEST(const std::string& filename);
    RegressionTreeEnsembleShapeEstimatorDEST(std::istream& is, const std::string& hint = {});

    virtual int operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const;
    virtual int operator()(const cv::Mat& I, Point2fVec& points, BoolVec& mask) const;
    virtual std::vector<cv::Point2f> getMeanShape() const;
    virtual void setDoPreview(bool flag) {}
    virtual bool isPCA() const;
    //virtual void setStagesHint(int stages);
    //virtual int getStagesHint() const;

protected:
    std::unique_ptr<Impl> m_impl;
};

DRISHTI_ML_NAMESPACE_END

#endif // __drishti_ml_RegressionTreeEnsembleShapeEstimatorDEST_h__
