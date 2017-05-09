/*!
  @file   RegressionTreeEnsembleShapeEstimator.cpp
  @author David Hirvonen
  @brief  Internal implementation of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/RTEShapeEstimatorImpl.h"

#if DRISHTI_SERIALIZE_WITH_BOOST
std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(const std::string& filename);
std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(std::istream& is);
#endif

// C++ exception with description:
// "Trying to save an unregistered polymorphic type (drishti::ml::RegressionTreeEnsembleShapeEstimator).
// Make sure your type is registered with CEREAL_REGISTER_TYPE and that the archive you are using was
// included (and registered with CEREAL_REGISTER_ARCHIVE) prior to calling CEREAL_REGISTER_TYPE.
// If your type is already registered and you still see this error, you may need to use
// CEREAL_REGISTER_DYNAMIC_INIT." thrown in the test body.

DRISHTI_ML_NAMESPACE_BEGIN

RTEShapeEstimator::Impl::Impl(const std::string& filename)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if ((filename.find(".pba.z") != std::string::npos) && !m_predictor)
    {
        m_predictor = load_pba_z(filename);
        return;
    }
#endif
}

RTEShapeEstimator::Impl::Impl(std::istream& is)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    // istream test breaks qt stream reading:
    if (/* is_pba_z(is) && */ !m_predictor)
    {
        m_predictor = load_pba_z(is);
        return;
    }
#endif
}

void RTEShapeEstimator::setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
{
    m_streamLogger = logger;
    if (m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(const std::string& filename)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if (filename.find(".pba.z") != std::string::npos)
    {
        // Legacy format (Impl serialization):
        m_impl = std::make_shared<RegressionTreeEnsembleShapeEstimator::Impl>(filename);
        return;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CEREAL
    if ((filename.find(".cpb") != std::string::npos))
    {
        load_cpb(filename, *this);
        return;
    }
#endif
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(std::istream& is, const std::string& hint)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if ((!hint.empty() && (hint.find(".pba.z") != std::string::npos)) || (hint.empty() && is_pba_z(is)))
    {
        // Legacy format (Impl serialization):
        m_impl = std::make_shared<RegressionTreeEnsembleShapeEstimator::Impl>(is);
        return;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CEREAL
    {
        load_cpb(is, *this);
        return;
    }
#endif
}

void RTEShapeEstimator::setStagesHint(int stages)
{
    DRISHTI_STREAM_LOG_FUNC(6, 7, m_streamLogger);
    m_impl->setStagesHint(stages);
}

int RTEShapeEstimator::getStagesHint() const
{
    DRISHTI_STREAM_LOG_FUNC(6, 8, m_streamLogger);
    return m_impl->getStagesHint();
}

int RTEShapeEstimator::operator()(const cv::Mat& gray, std::vector<cv::Point2f>& points, std::vector<bool>& mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6, 9, m_streamLogger);
    return (*m_impl)(gray, points, mask);
}

int RTEShapeEstimator::operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6, 10, m_streamLogger);
    CV_Assert(false);
    return 0;
}

bool RTEShapeEstimator::isPCA() const
{
    DRISHTI_STREAM_LOG_FUNC(6, 11, m_streamLogger);
    return m_impl->isPCA();
}

std::vector<cv::Point2f> RTEShapeEstimator::getMeanShape() const
{
    return m_impl->getMeanShape();
}

void RTEShapeEstimator::dump(std::vector<float> &values, bool pca)
{
    return m_impl->dump(values, pca);
}

DRISHTI_ML_NAMESPACE_END

// http://www.boost.org/doc/libs/1_46_1/libs/serialization/doc/special.html
// Including BOOST_CLASS_EXPORT_IMPLEMENT in multiple files could result in a failure to link due to
// duplicated symbols or the throwing of a runtime exception.

//#if DRISHTI_SERIALIZE_WITH_BOOST
//#  include "RTEShapeEstimatorArchiveBoost.cpp"
//#endif
