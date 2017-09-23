/**
  @file   FaceTracker.cpp
  @author David Hirvonen
  @brief  Implementation of public API for eye model estimation using simple portable types.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

 */

#include "drishti/FaceTracker.hpp"
#include "drishti/ContextImpl.h"
#include "drishti/FaceMonitorAdapter.h"
#include "drishti/SensorImpl.h"

#include "drishti/face/Face.h"
#include "drishti/hci/FaceFinder.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/Logger.h"

#include "drishti/drishti_cv.hpp"

#include <string>
#include <fstream>
#include <iostream>
#include <iomanip>
#include <sstream>
#include <memory>

_DRISHTI_SDK_BEGIN

static ogles_gpgpu::FrameInput convert(const VideoFrame& frame);

/*
 * Impl
 */

struct FaceTracker::Impl
{
    using Settings = drishti::hci::FaceFinder::Settings;

    /*
     * WIP: We will need to provide some configurable options here
     */

    Impl(Context* manager, FaceTracker::Resources& resources)
    {
        Settings settings;
        settings.sensor = manager->get()->sensor;

        if (resources.logger.empty())
        {
            settings.logger = drishti::core::Logger::create("drishti");
            settings.logger->set_level(spdlog::level::off); // by default...
        }
        else
        {
            settings.logger = drishti::core::Logger::create(resources.logger.c_str());
        }

        settings.threads = std::make_shared<tp::ThreadPool<>>();
        settings.outputOrientation = 0;
        settings.frameDelay = 1;
        settings.doLandmarks = true;
        settings.doFlow = false;
        settings.doBlobs = false;
        settings.doSingleFace = manager->getDoSingleFace();
        settings.minDetectionDistance = manager->getMinDetectionDistance();
        settings.maxDetectionDistance = manager->getMaxDetectionDistance();
        settings.faceFinderInterval = manager->getFaceFinderInterval();
        settings.acfCalibration = manager->getAcfCalibration();
        settings.regressorCropScale = manager->getRegressorCropScale();
        settings.minTrackHits = manager->getMinTrackHits();
        settings.maxTrackMisses = manager->getMaxTrackMisses();
        settings.minFaceSeparation = manager->getMinFaceSeparation();
        settings.doOptimizedPipeline = manager->getDoOptimizedPipeline();

        auto stream = std::make_shared<drishti::face::FaceDetectorFactoryStream>();
        stream->iEyeRegressor = resources.sEyeRegressor;
        stream->iFaceDetector = resources.sFaceDetector;
        stream->iFaceRegressor = resources.sFaceRegressor;
        stream->iFaceDetectorMean = resources.sFaceModel;

        std::shared_ptr<drishti::face::FaceDetectorFactory> factory = stream;

        m_faceFinder = drishti::hci::FaceFinder::create(factory, settings, manager->get()->glContext);
    }

    int operator()(const VideoFrame& frame)
    {
        return (*m_faceFinder)(convert(frame));
    }

    void add(drishti_face_tracker_t& table)
    {
        auto callback = std::make_shared<FaceMonitorAdapter>(table);
        m_faceFinder->registerFaceMonitorCallback(callback.get());
        m_callbacks.emplace_back(callback);
    }

    std::vector<std::shared_ptr<FaceMonitorAdapter>> m_callbacks;

    std::unique_ptr<drishti::hci::FaceFinder> m_faceFinder;
};

/*
 * FaceTracker
 */

int FaceTracker::operator()(const VideoFrame& image)
{
    return (*m_impl)(image);
}

FaceTracker::FaceTracker(Context* manager, Resources& factory)
{
    m_impl = drishti::core::make_unique<Impl>(manager, factory);
}

FaceTracker::~FaceTracker()
{
}

bool FaceTracker::good() const
{
    return static_cast<bool>(m_impl.get());
}

FaceTracker::operator bool() const
{
    return good();
}

void FaceTracker::tryEnablePlatformOptimizations()
{
    drishti::hci::FaceFinder::tryEnablePlatformOptimizations();
}

void FaceTracker::add(drishti_face_tracker_t& table)
{
    m_impl->add(table);
}

// ### utility

static ogles_gpgpu::FrameInput convert(const VideoFrame& frame)
{
    // clang-format off
    ogles_gpgpu::FrameInput input
    {
        { frame.size[0], frame.size[1] },
        frame.pixelBuffer,
        frame.useRawPixels,
        frame.inputTexture,
        frame.textureFormat
    };
    // clang-format on

    return input;
}

_DRISHTI_SDK_END

/*
 * Extern "C" interface (dlopen/dlsym)
 */

DRISHTI_EXTERN_C_BEGIN

DRISHTI_EXPORT drishti::sdk::FaceTracker*
drishti_face_tracker_create_from_streams(drishti::sdk::Context* manager, drishti::sdk::FaceTracker::Resources& resources)
{
    return new drishti::sdk::FaceTracker(manager, resources);
}

DRISHTI_EXPORT void
drishti_face_tracker_destroy(drishti::sdk::FaceTracker* tracker)
{
    if (tracker)
    {
        delete tracker;
    }
}

DRISHTI_EXPORT int
drishti_face_tracker_callback(drishti::sdk::FaceTracker* tracker, drishti_face_tracker_t& table)
{
    if (tracker)
    {
        tracker->add(table);
    }
    return -1;
}

DRISHTI_EXPORT int
drishti_face_tracker_track(drishti::sdk::FaceTracker* tracker, const drishti::sdk::VideoFrame& image)
{
    if (tracker)
    {
        return (*tracker)(image);
    }
    return -1;
}

DRISHTI_EXTERN_C_END
