/*! -*-c++-*-
 @file   drishti/hci/FaceFinderImpl.h
 @author David Hirvonen
 @brief  Face detection and tracking class with GPU acceleration.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __drishti_hci_FaceFinderImpl_h__
#define __drishti_hci_FaceFinderImpl_h__

#include "drishti/hci/drishti_hci.h"

#include "drishti/core/Logger.h"              // spdlog::logger
#include "drishti/eye/gpu/EllipsoPolarWarp.h" // ogles_gpgpu::EllipsoPolarWarp
#include "drishti/eye/gpu/EyeWarp.h"
#include "drishti/face/gpu/EyeFilter.h"       // ogles_gpgpu::EyeFilter
#include "drishti/face/FaceDetector.h"        // drishti::face::FaceDetector
#include "drishti/face/FaceDetectorFactory.h" // drishti::face::FaceDetectorFactory
#include "drishti/face/FaceModelEstimator.h"  // drishti::face::FaceModelEstimator
#include "drishti/face/FaceTracker.h"         // drishti::face::FaceTracker
#include "drishti/hci/FaceMonitor.h"          // FaceMonitor*
#include "drishti/hci/Scene.hpp"              // ScenePrimitives
#include "drishti/hci/gpu/BlobFilter.h"       // ogles_gpgpu::BlobFilter
#include "drishti/sensor/Sensor.h"            // drishti::sensor::SensorModel

#include <acf/ACF.h>                          // drishti::acf::Detector+Pyramid
#include <acf/GPUACF.h>               // ogles_gpgpu::ACF

#include "ogles_gpgpu/common/proc/swizzle.h"   // ogles_gpgpu::SwizzleProc
#include "ogles_gpgpu/common/proc/flow.h"      // ogles_gpgpu::FlowOptPipeline
#include "ogles_gpgpu/common/proc/fifo.h"      // ogles_gpgpu::FifoProc
#include "ogles_gpgpu/common/proc/transform.h" // ogles_gpgpu::TransformProc
#include "thread_pool/thread_pool.hpp"         // tp::ThreadPool<>

#include <chrono> // std::chrono::high_resolution_clock::time_point
#include <future> // future
#include <memory> // std::shared_ptr
#include <vector> // vector

#define DRISHTI_HCI_FACEFINDER_LANDMARKS_WIDTH 1024
#define DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT 1 // *** display ***
#define DRISHTI_HCI_FACEFINDER_DO_TRACKING 1
#define DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES 1
#define DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES_DISPLAY 1
#define DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS 0
#define DRISHTI_HCI_FACEFINDER_LOG_DETECTIONS 0
#define DRISHTI_HCI_FACEFINDER_ENABLE_ACF_FILTER_LOGGING 0

DRISHTI_HCI_NAMESPACE_BEGIN

struct FaceFinder::Impl
{
    Impl(std::shared_ptr<drishti::face::FaceDetectorFactory>& factory, Settings& args, void* glContext)
        : glContext(glContext)
        , factory(factory)
        , sensor(args.sensor)
        , logger(args.logger)
        , threads(args.threads)
        , hasInit(false)
        , outputOrientation(args.outputOrientation)

        // ACF and detection parameters:
        , debugACF(false)
        , acfCalibration(args.acfCalibration)
        , doSingleFace(args.doSingleFace)
        , faceFinderInterval(args.faceFinderInterval)
        , minDistanceMeters(args.minDetectionDistance)
        , maxDistanceMeters(args.maxDetectionDistance)
        , minTrackHits(args.minTrackHits)
        , maxTrackMisses(args.maxTrackMisses)
        , minFaceSeparation(args.minFaceSeparation)

        // Face landmarks:
        , doLandmarks(args.doLandmarks)
        , landmarksWidth(DRISHTI_HCI_FACEFINDER_LANDMARKS_WIDTH)
        , regressorCropScale(args.regressorCropScale)

        // Eye parameters:
        , doBlobs(args.doBlobs)
        , doIris(DRISHTI_HCI_FACEFINDER_DO_ELLIPSO_POLAR)
        , eyesSize(args.eyesSize)
        , doEyesScaling(args.doEyesScaling)

        // Annotations:
        , renderFaces(args.renderFaces)
        , renderPupils(args.renderPupils)
        , renderCorners(args.renderCorners)
        , renderEyesWidthRatio(args.renderEyesWidthRatio)

        , glVersionMajor(args.glVersionMajor)
        , glVersionMinor(args.glVersionMinor)
        , usePBO(args.usePBO)
        , doOptimizedPipeline(args.doOptimizedPipeline)
        , history(args.history)
    {
    }

    using time_point = std::chrono::high_resolution_clock::time_point;

    // :::::::::::::::::::::::
    // ::: Core parameters :::
    // :::::::::::::::::::::::
    void* glContext = nullptr;
    std::shared_ptr<drishti::face::FaceDetectorFactory> factory;
    std::shared_ptr<drishti::sensor::SensorModel> sensor;
    std::shared_ptr<spdlog::logger> logger;
    std::shared_ptr<tp::ThreadPool<>> threads;
    std::vector<FaceMonitor*> faceMonitorCallback;
    ImageLogger imageLogger;
    TimePoint start;
    TimerInfo timerInfo;

    bool doAnnotations = true;
    bool hasInit = false;
    int index = 0;
    uint64_t frameIndex = 0;
    cv::Mat3f colors32FC3; // map angles to colors

    // :::::::::::::::::::::::::::::::::::::::
    // ::: Input frame related parameters: :::
    // :::::::::::::::::::::::::::::::::::::::

    int outputOrientation = 0;
    float brightness = 1.f;
    std::shared_ptr<ogles_gpgpu::FifoProc> fifo; // store last N faces

    // :::::::::::::::::::::::::::::::::::::::
    // ::: ACF and detection parameters:   :::
    // :::::::::::::::::::::::::::::::::::::::

    // ACF:
    bool debugACF = false;
    bool doCpuACF = false;
    float ACFScale = 2.0f;
    std::vector<cv::Size> pyramidSizes;
    acf::Detector::Pyramid P;
    std::shared_ptr<ogles_gpgpu::ACF> acf;
    float acfCalibration = 0.f;

    // Detection:
    bool doSingleFace = false;
    double faceFinderInterval = DRISHTI_HCI_FACEFINDER_INTERVAL;
    float minDistanceMeters = 0.f;
    float maxDistanceMeters = 10.0f;
    std::size_t minTrackHits = 3;
    std::size_t maxTrackMisses = 3;
    float minFaceSeparation = 0.15;
    std::unique_ptr<drishti::face::FaceDetector> faceDetector;
    std::unique_ptr<drishti::face::FaceTracker> faceTracker;

    acf::Detector* detector = nullptr; // weak ref
    std::pair<time_point, std::vector<cv::Rect>> objects;
    std::future<ScenePrimitives> scene;
    std::deque<ScenePrimitives> scenePrimitives; // stash

    // ::::::::::::::::::::::::::::::::::::::::
    // ::: Face landmark parameters 2d->3d: :::
    // ::::::::::::::::::::::::::::::::::::::::
    bool doLandmarks = false;
    int landmarksWidth = 256;
    float regressorCropScale = 0.f;

    // Camera model, etc:
    cv::Point3f faceMotion;
    std::shared_ptr<drishti::face::FaceModelEstimator> faceEstimator; // pinhole model

    // :::::::::::::::::::::::
    // ::: Eye parameters: :::
    // :::::::::::::::::::::::

    bool doBlobs = false;
    bool doIris = false;
    bool doEyeFlow = false;
    cv::Size eyesSize = { 480, 240 };
    bool doEyesScaling = true;

    std::unique_ptr<ogles_gpgpu::BlobFilter> blobFilter;
    std::unique_ptr<ogles_gpgpu::EyeFilter> eyeFilter;
    std::shared_ptr<ogles_gpgpu::EllipsoPolarWarp> ellipsoPolar[2];
    std::unique_ptr<ogles_gpgpu::FlowOptPipeline> eyeFlow;
    std::unique_ptr<ogles_gpgpu::SwizzleProc> eyeFlowBgra; // (optional)
    ogles_gpgpu::ProcInterface* eyeFlowBgraInterface = nullptr;

    FeaturePoints gazePoints;
    std::array<FeaturePoints, 2> eyePoints;

    cv::Point2f eyeMotion;
    std::vector<cv::Vec4f> eyeFlowField;

    // ::::::::::::::::::::
    // ::: Annotations  :::
    // ::::::::::::::::::::
    bool renderFaces = true;
    bool renderPupils = true;
    bool renderCorners = true;
    float renderEyesWidthRatio = 0.25f;

    int glVersionMajor = 2;
    int glVersionMinor = 0;
    bool usePBO = false;
    bool doOptimizedPipeline = true;
    int history = 3; // frame history
    int latency = 2;
    
    // :::::::::::::::::::::::
    // ::: Filters/Effects :::
    // :::::::::::::::::::::::
    std::unique_ptr<ogles_gpgpu::TransformProc> warper;
    std::unique_ptr<ogles_gpgpu::TransformProc> rotater; // output frame rotation
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceFinderImpl_h__
