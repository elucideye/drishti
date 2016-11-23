/*!
  @file   RegressionTreeEnsembleShapeEstimator.h
  @author David Hirvonen
  @brief  Internal declaration of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_RegressionTreeEnsembleShapeEstimator_h
#define drishtisdk_RegressionTreeEnsembleShapeEstimator_h

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

    RegressionTreeEnsembleShapeEstimator() {}
    RegressionTreeEnsembleShapeEstimator(const std::string &filename);
    RegressionTreeEnsembleShapeEstimator(std::istream &is);

    virtual void setStreamLogger(std::shared_ptr<spdlog::logger> &logger);
    virtual int operator()(const cv::Mat &I, const cv::Mat &M, Point2fVec &points, BoolVec &mask) const;
    virtual int operator()(const cv::Mat &I, Point2fVec &points, BoolVec &mask) const;
    virtual std::vector<cv::Point2f> getMeanShape() const;
    virtual void setDoPreview(bool flag) {}
    virtual bool isPCA() const;
    virtual void setStagesHint(int stages);
    virtual int getStagesHint() const;

    void saveImpl(const std::string &filename);
    void loadImpl(const std::string &filename);

    // Boost serialization:
    template<class Archive> void serialize(Archive & ar, const unsigned int version);
    template< class Archive> void serializeModel(Archive &ar, const unsigned int version);

    std::shared_ptr<Impl> m_impl;
};

typedef RegressionTreeEnsembleShapeEstimator RTEShapeEstimator;

DRISHTI_ML_NAMESPACE_END

#include "drishti/core/boost_serialize_common.h"

BOOST_CLASS_EXPORT_KEY(drishti::ml::RegressionTreeEnsembleShapeEstimator);
BOOST_CLASS_EXPORT_KEY(drishti::ml::RegressionTreeEnsembleShapeEstimator::Impl);

#endif // drishtisdk_RegressionTreeEnsembleShapeEstimator_h
