/*! -*-c++-*-
  @file   RegressionTreeEnsembleShapeEstimator.h
  @author David Hirvonen
  @brief  Internal declaration of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_ml_RegressionTreeEnsembleShapeEstimator_h__
#define __drishti_ml_RegressionTreeEnsembleShapeEstimator_h__

#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/ShapeEstimator.h"

#include <opencv2/core/core.hpp>

#include <memory>
#include <vector>

DRISHTI_ML_NAMESPACE_BEGIN

// Consider occlusion estimation
class RegressionTreeEnsembleShapeEstimator : public ShapeEstimator
{
public:
    class Impl;

    RegressionTreeEnsembleShapeEstimator();
    RegressionTreeEnsembleShapeEstimator(const std::string& filename);
    RegressionTreeEnsembleShapeEstimator(std::istream& is, const std::string& hint = {});
    ~RegressionTreeEnsembleShapeEstimator() override;

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger) override;
    int operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const override;
    int operator()(const cv::Mat& I, Point2fVec& points, BoolVec& mask) const override;
    std::vector<cv::Point2f> getMeanShape() const override;
    void setDoPreview(bool flag) override {}
    bool isPCA() const override;

    void setStagesHint(int stages) override;
    int getStagesHint() const override;

    void dump(std::vector<float>& values, bool pca) override;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    template <class Archive>
    void serializeModel(Archive& ar, const unsigned int version);

protected:
    std::unique_ptr<Impl> m_impl;
};

using RTEShapeEstimator = drishti::ml::RegressionTreeEnsembleShapeEstimator;

DRISHTI_ML_NAMESPACE_END

// NOTE: Initial attempt to move this code to the RTEShapeEstimatorArchiveCereal.cpp failed
#include "drishti/core/drishti_stdlib_string.h"
#include <cereal/types/polymorphic.hpp>
CEREAL_REGISTER_TYPE(drishti::ml::RegressionTreeEnsembleShapeEstimator);
CEREAL_REGISTER_POLYMORPHIC_RELATION(drishti::ml::ShapeEstimator, drishti::ml::RegressionTreeEnsembleShapeEstimator);

#endif // __drishti_ml_RegressionTreeEnsembleShapeEstimator_h__
