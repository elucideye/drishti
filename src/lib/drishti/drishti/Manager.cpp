/**
  @file   drishti/Manager.hpp
  @author David Hirvonen
  @brief  Public API for continuous face filter.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/Manager.hpp"

#include "drishti/hci/FaceFinder.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/Logger.h" // spdlog::logger
#include "drishti/sensor/Sensor.h" 
#include "thread_pool/thread_pool.hpp"

_DRISHTI_SDK_BEGIN

class Manager::Impl
{
public:
    
    Impl(const drishti::sensor::SensorModel &sensor)
    {
        m_sensor = std::make_shared<drishti::sensor::SensorModel>(sensor);
        m_logger = drishti::core::Logger::create("drishti");
        m_threads = std::make_shared<ThreadPool<128>>(); // thread-pool
    }
    
    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<ThreadPool<128>> m_threads; 
};

Manager::Manager(const drishti::sensor::SensorModel &sensor)
{
    m_impl = drishti::core::make_unique<Impl>(sensor);
}

_DRISHTI_SDK_END

