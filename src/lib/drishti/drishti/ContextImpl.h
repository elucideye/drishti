/**
  @file   drishti/ContextImpl.h
  @author David Hirvonen
  @brief  Private header for the Context class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_drishti_ContextImpl_h__
#define __drishti_drishti_ContextImpl_h__ 1

#include "drishti/Context.hpp"

#include "drishti/hci/FaceFinder.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/Logger.h" // spdlog::logger
#include "drishti/sensor/Sensor.h"

#include "thread_pool/thread_pool.hpp"

#define DRISHTI_LOGGER_NAME "drishti"

_DRISHTI_SDK_BEGIN

#define DEFAULT_MIN_DETECTION_DISTANCE 0.0
#define DEFAULT_MAX_DETECTION_DISTANCE 1.0

class Context::Impl
{
public:
    Impl(const drishti::sensor::SensorModel &sensor);

    std::shared_ptr<drishti::sensor::SensorModel> & getSensor() { return m_sensor; }
    std::shared_ptr<spdlog::logger> & getLogger() { return m_logger; }
    std::shared_ptr<tp::ThreadPool<>> & getThreads() { return m_threads; }
    void * getGlContext() { return m_glContext; }
    
    float getMinDetectionDistance() const { return m_minDetectionDistance; }
    void setMinDetectionDistance(float value) { m_minDetectionDistance = value; }
    
    float getMaxDetectionDistance() const { return m_maxDetectionDistance; }
    void setMaxDetectionDistance(float value) { m_maxDetectionDistance = value; }

protected:
    
    float m_minDetectionDistance = DEFAULT_MIN_DETECTION_DISTANCE;
    float m_maxDetectionDistance = DEFAULT_MAX_DETECTION_DISTANCE;
    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<tp::ThreadPool<>> m_threads;
    void * m_glContext = nullptr;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_ContextImpl_h__ 
