/*! -*-c++-*-
  @file   XGBooster.cpp
  @author David Hirvonen
  @brief  Internal implementation of the XGBoost C++ interface class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/XGBooster.h"
#include "drishti/ml/XGBoosterImpl.h"

DRISHTI_ML_NAMESPACE_BEGIN

XGBooster::Impl::~Impl() = default;

XGBooster::XGBooster()
{
    m_impl = drishti::core::make_unique<XGBooster::Impl>();
}

XGBooster::XGBooster(const Recipe& recipe)
{
    m_impl = drishti::core::make_unique<XGBooster::Impl>(recipe);
}

XGBooster::~XGBooster() = default;

void XGBooster::setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
{
    m_streamLogger = logger;
    if (m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

float XGBooster::operator()(const std::vector<float>& features)
{
    return (*m_impl)(features);
}

void XGBooster::train(const MatrixType<float>& features, const std::vector<float>& values, const MatrixType<uint8_t>& mask)
{
#if DRISHTI_BUILD_MIN_SIZE
    assert(false);
#else
    m_impl->train(features, values, mask);
#endif
}

void XGBooster::read(const std::string& filename)
{
#if DRISHTI_BUILD_MIN_SIZE
    assert(false);
#else
    m_impl->read(filename);
#endif
}
void XGBooster::write(const std::string& filename) const
{
#if DRISHTI_BUILD_MIN_SIZE
    assert(false);
#else
    m_impl->write(filename);
#endif
}

DRISHTI_ML_NAMESPACE_END
