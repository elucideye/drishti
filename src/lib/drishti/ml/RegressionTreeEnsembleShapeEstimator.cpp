/*! -*-c++-*-
  @file   RegressionTreeEnsembleShapeEstimator.cpp
  @author David Hirvonen
  @brief  Internal implementation of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/RTEShapeEstimatorImpl.h"

// C++ exception with description:
// "Trying to save an unregistered polymorphic type (drishti::ml::RegressionTreeEnsembleShapeEstimator).
// Make sure your type is registered with CEREAL_REGISTER_TYPE and that the archive you are using was
// included (and registered with CEREAL_REGISTER_ARCHIVE) prior to calling CEREAL_REGISTER_TYPE.
// If your type is already registered and you still see this error, you may need to use
// CEREAL_REGISTER_DYNAMIC_INIT." thrown in the test body.

DRISHTI_ML_NAMESPACE_BEGIN

// ############
// ### Impl ###
// ############

RTEShapeEstimator::Impl::Impl() = default;
RTEShapeEstimator::Impl::~Impl() = default;

RTEShapeEstimator::Impl::Impl(const std::string& filename)
{
    m_predictor = make_unique_cpb<_SHAPE_PREDICTOR>(filename);
}

RTEShapeEstimator::Impl::Impl(std::istream& is, const std::string& /*hint*/)
{
    m_predictor = make_unique_cpb<_SHAPE_PREDICTOR>(is);
}

// ############################################
// ### RegressionTreeEnsembleShapeEstimator ###
// ############################################

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator() = default;
RTEShapeEstimator::~RegressionTreeEnsembleShapeEstimator() = default;

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(const std::string& filename)
{
    m_impl = drishti::core::make_unique<Impl>(filename);
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(std::istream& is, const std::string& hint)
{
    m_impl = drishti::core::make_unique<Impl>(is, hint);
}

void RTEShapeEstimator::setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
{
    m_streamLogger = logger;
    if (m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

void RTEShapeEstimator::setStagesHint(int stages)
{
    m_impl->setStagesHint(stages);
}

int RTEShapeEstimator::getStagesHint() const
{
    return m_impl->getStagesHint();
}

int RTEShapeEstimator::operator()(const cv::Mat& gray, std::vector<cv::Point2f>& points, std::vector<bool>& mask) const
{
    return (*m_impl)(gray, points, mask);
}

int RTEShapeEstimator::operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const
{
    CV_Assert(false);
    return 0;
}

bool RTEShapeEstimator::isPCA() const
{
    return m_impl->isPCA();
}

std::vector<cv::Point2f> RTEShapeEstimator::getMeanShape() const
{
    return m_impl->getMeanShape();
}

void RTEShapeEstimator::dump(std::vector<float>& values, bool pca)
{
    return m_impl->dump(values, pca);
}

DRISHTI_ML_NAMESPACE_END
