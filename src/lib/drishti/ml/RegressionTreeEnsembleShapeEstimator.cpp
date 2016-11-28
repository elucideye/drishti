/*!
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

void RTEShapeEstimator::setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
{
    m_streamLogger = logger;
    if(m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(const std::string &filename)
{
    m_impl = std::make_shared<RegressionTreeEnsembleShapeEstimator::Impl>(filename);
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(std::istream &is)
{
    m_impl = std::make_shared<RegressionTreeEnsembleShapeEstimator::Impl>(is);
}

void RTEShapeEstimator::setStagesHint(int stages)
{
    DRISHTI_STREAM_LOG_FUNC(6,7,m_streamLogger);
    m_impl->setStagesHint(stages);
}

int RTEShapeEstimator::getStagesHint() const
{
    DRISHTI_STREAM_LOG_FUNC(6,8,m_streamLogger);
    return m_impl->getStagesHint();
}

int RTEShapeEstimator::operator()(const cv::Mat &gray, std::vector<cv::Point2f> &points, std::vector<bool> &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6,9,m_streamLogger);
    return (*m_impl)(gray, points, mask);
}

int RTEShapeEstimator::operator()(const cv::Mat &I, const cv::Mat &M, Point2fVec &points, BoolVec &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6,10,m_streamLogger);
    CV_Assert(false);
    return 0;
}

bool RTEShapeEstimator::isPCA() const
{
    DRISHTI_STREAM_LOG_FUNC(6,11,m_streamLogger);
    return m_impl->isPCA();
}

std::vector<cv::Point2f> RTEShapeEstimator::getMeanShape() const
{
    return m_impl->getMeanShape();
}

DRISHTI_ML_NAMESPACE_END

// http://www.boost.org/doc/libs/1_46_1/libs/serialization/doc/special.html
// Including BOOST_CLASS_EXPORT_IMPLEMENT in multiple files could result in a failure to link due to
// duplicated symbols or the throwing of a runtime exception.

#if DRISHTI_SERIALIZE_WITH_BOOST 
#  include "RTEShapeEstimatorArchiveBoost.cpp"
#endif
