/*!
  @file   drishti/hci/FaceFinder.h
  @author David Hirvonen
  @brief  Face detection and tracking class with GPU acceleration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_FaceFinder_h__
#define __drishti_hci_FaceFinder_h__

#include "drishti/hci/drishti_hci.h"
#include "drishti/hci/Scene.hpp"
#include "drishti/hci/FaceMonitor.h"
#include "drishti/acf/GPUACF.h"
#include "drishti/acf/ACF.h"
#include "drishti/face/Face.h"
#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/sensor/Sensor.h"

#include "ogles_gpgpu/common/proc/flow.h"

#include "thread_pool/thread_pool.hpp"

#include <memory>
#include <chrono>
#include <functional>

#define DRISHTI_HCI_FACEFINDER_INTERVAL 0.1
#define DRISHTI_HCI_FACEFINDER_DO_ELLIPSO_POLAR 0

// *INDENT-OFF*
namespace ogles_gpgpu
{
    class FlashFilter;
    class FacePainter;
    class FifoProc;
    class TransformProc;
    class EyeFilter;
    class EllipsoPolarWarp;
}

namespace spdlog { class logger; }

namespace drishti
{
    namespace sensor
    {
        class Sensor;
    }
    namespace face
    {
        class FaceModelEstimator;
        class FaceDetector;
    }
}
// *INDENT-ON*

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceFinder
{
public:

    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point;// <std::chrono::system_clock>;
    using FrameInput = ogles_gpgpu::FrameInput;

    using FaceDetectorFactoryPtr = std::shared_ptr<drishti::face::FaceDetectorFactory>;
    
    struct TimerInfo
    {
        double detectionTime;
        double regressionTime;
        double eyeRegressionTime;
        std::function<void(double second)> detectionTimeLogger;
        std::function<void(double second)> regressionTimeLogger;
        std::function<void(double second)> eyeRegressionTimeLogger;
    };
    
    struct Config
    {
        std::shared_ptr<drishti::sensor::SensorModel> sensor;
        std::shared_ptr<spdlog::logger> logger;
        std::shared_ptr<ThreadPool<128>> threads;        
        int outputOrientation = 0;
        int frameDelay = 1;
        bool doLandmarks = true;
        bool doFlow = true;
        bool doFlash = false;
    };

    FaceFinder(FaceDetectorFactoryPtr &factory, Config &config, void *glContext = nullptr);

    virtual GLuint operator()(const FrameInput &frame);
    
    void setMaxDistance(float meters);
    float getMaxDistance() const;
    
    void setMinDistance(float meters);
    float getMinDistance() const;

    void setDoCpuAcf(bool flag);
    bool getDoCpuAcf() const;

    void setFaceFinderInterval(double interval);
    double getFaceFinderInterval() const;
    
    void registerFaceMonitorCallback(FaceMonitor *callback);
    
protected:
    
    void updateEyes(GLuint inputTexId, const ScenePrimitives &scene);
    
    void notifyListeners(const ScenePrimitives &scene, const TimePoint &time, bool isFull);
    bool hasValidFaceRequest(const ScenePrimitives &scene, const TimePoint &time) const;

    virtual void init(const FrameInput &frame);
    virtual void initPainter(const cv::Size &inputSizeUp);
    
    void initACF(const cv::Size &inputSizeUp);
    void initFIFO(const cv::Size &inputSize);
    void initFlasher();
    void initColormap(); // [0..359];
    void initEyeEnhancer(const cv::Size &inputSizeUp, const cv::Size &eyesSize);
    void initIris(const cv::Size &size);
    
    void init2(drishti::face::FaceDetectorFactory &resources);
    void detect2(const FrameInput &frame, ScenePrimitives &scene);

    void dumpEyes(std::vector<cv::Mat4b> &frames);
    void dumpFaces(std::vector<cv::Mat4b> &frames);
    virtual int detect(const FrameInput &frame, ScenePrimitives &scene);
    virtual GLuint paint(const ScenePrimitives &scene, GLuint inputTexture);
    virtual void preprocess(const FrameInput &frame, ScenePrimitives &scene); // compute acf
    int computeDetectionWidth(const cv::Size &inputSizeUp) const;
    
    std::shared_ptr<acf::Detector::Pyramid> createAcfGpu(const FrameInput &frame);
    std::shared_ptr<acf::Detector::Pyramid> createAcfCpu(const FrameInput &frame);
    void fill(drishti::acf::Detector::Pyramid &P);

    cv::Mat3f m_colors32FC3; // map angles to colors

    void *m_glContext = nullptr;
    bool m_hasInit = false;
    float m_ACFScale = 2.0f;
    int m_outputOrientation = 0;

    uint64_t m_frameIndex = 0;

    using time_point = std::chrono::high_resolution_clock::time_point;
    std::pair<time_point, std::vector<cv::Rect>> m_objects;

    drishti::acf::Detector::Pyramid m_P;

    double m_faceFinderInterval = DRISHTI_HCI_FACEFINDER_INTERVAL;

    float m_minDistanceMeters = 0.f;
    float m_maxDistanceMeters = 10.0f;

    bool m_doLandmarks = false;
    int m_landmarksWidth = 256;

    bool m_doFlow = false;
    int m_flowWidth = 256;

    bool m_doFlash = false;
    int m_flashWidth = 128;
    
    bool m_doIris = false;
    
    bool m_doCpuACF= false;
    
    cv::Size m_eyesSize = { 480, 240 };
    
    std::vector<cv::Size> m_pyramidSizes;

    drishti::acf::Detector *m_detector = nullptr; // weak ref

    std::shared_ptr<drishti::face::FaceDetector> m_faceDetector;
    
    // Model estimator from pinhole camera model:
    std::shared_ptr<drishti::face::FaceModelEstimator> m_faceEstimator;
    
    std::shared_ptr<ogles_gpgpu::ACF> m_acf;
    std::shared_ptr<ogles_gpgpu::FifoProc> m_fifo;
    std::shared_ptr<ogles_gpgpu::FacePainter> m_painter;
    std::shared_ptr<ogles_gpgpu::TransformProc> m_rotater; // For QT
    std::shared_ptr<ogles_gpgpu::FlashFilter> m_flasher; // EXPERIMENTAL
    std::shared_ptr<ogles_gpgpu::EyeFilter> m_eyeFilter;
    std::shared_ptr<ogles_gpgpu::EllipsoPolarWarp> m_ellipsoPolar[2];

    int m_index = 0;
    std::vector<std::future<ScenePrimitives>> m_scenes;

    TimerInfo m_timerInfo;
    
    std::shared_ptr<drishti::face::FaceDetectorFactory> m_factory;
    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<ThreadPool<128>> m_threads;
    
    TimePoint m_start;
    
    std::vector<FaceMonitor*> m_faceMonitorCallback;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceFinder_h__
