/**
  @file   drishti/Context.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/Context.hpp"
#include "drishti/ContextImpl.h"

#define DRISHTI_LOGGER_NAME "drishti"

_DRISHTI_SDK_BEGIN

Context::Impl::Impl(const drishti::sensor::SensorModel &sensor)
{
    m_sensor = std::make_shared<drishti::sensor::SensorModel>(sensor);
    m_logger = drishti::core::Logger::create(DRISHTI_LOGGER_NAME);
    m_threads = std::make_shared<tp::ThreadPool<>>(); // thread-pool
}

Context::Context(const drishti::sensor::SensorModel &sensor)
{
    m_impl = drishti::core::make_unique<Impl>(sensor);
}

Context::~Context()
{

}

void Context::setMinDetectionDistance(float value)
{
    return m_impl->setMinDetectionDistance(value);
}

float Context::getMinDetectionDistance() const
{
    return m_impl->getMinDetectionDistance();
}

void Context::setMaxDetectionDistance(float value)
{
    return m_impl->setMaxDetectionDistance(value);
}

float Context::getMaxDetectionDistance() const
{
    return m_impl->getMaxDetectionDistance();
}

_DRISHTI_SDK_END

