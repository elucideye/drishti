/**
  @file   drishti/Manager.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/Manager.hpp"
#include "drishti/ManagerImpl.h"

#define DRISHTI_LOGGER_NAME "drishti"

_DRISHTI_SDK_BEGIN

Manager::Impl::Impl(const drishti::sensor::SensorModel &sensor)
{
    m_sensor = std::make_shared<drishti::sensor::SensorModel>(sensor);
    m_logger = drishti::core::Logger::create(DRISHTI_LOGGER_NAME);
    m_threads = std::make_shared<ThreadPool<128>>(); // thread-pool
}

Manager::Manager(const drishti::sensor::SensorModel &sensor)
{
    m_impl = drishti::core::make_unique<Impl>(sensor);
}

_DRISHTI_SDK_END

