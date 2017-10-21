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
#define DEFAULT_MIN_TRACK_AGE 3

struct Context::Impl
{
    Impl(drishti::sdk::SensorModel& sensor);

    bool doSingleFace = true;
    float minDetectionDistance = DEFAULT_MIN_DETECTION_DISTANCE;
    float maxDetectionDistance = DEFAULT_MAX_DETECTION_DISTANCE;
    float faceFinderInterval = 0.f;
    float acfCalibration = 0.f;
    float regressorCropScale = 0.f;
    int minTrackHits = 0;
    int maxTrackMisses = 1;
    float minFaceSeparation = 0.125f;
    bool doOptimizedPipeline = false;

    std::shared_ptr<drishti::sensor::SensorModel> sensor;
    std::shared_ptr<spdlog::logger> logger;
    std::shared_ptr<tp::ThreadPool<>> threads;
    void* glContext = nullptr;
};

_DRISHTI_SDK_END

#endif // __drishti_drishti_ContextImpl_h__
