/*!
  @file   finder/FaceFinder.h
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/
#ifndef FACE_FINDER_H
#define FACE_FINDER_H

#include "drishti/acf/GPUACF.h"
#include "drishti/acf/ACF.h"
#include "drishti/face/Face.h"
#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/sensor/Sensor.h"

#include "Scene.hpp"

#include "ogles_gpgpu/common/proc/flow.h"

#include "thread_pool/thread_pool.hpp"

#include <memory>
#include <chrono>

#define DRISHTI_FACEFILTER_DO_ELLIPSO_POLAR 0

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

struct TimerInfo
{
    double detectionTime;
    double regressionTime;
    double eyeRegressionTime;
    std::function<void(double second)> detectionTimeLogger;
    std::function<void(double second)> regressionTimeLogger;
    std::function<void(double second)> eyeRegressionTimeLogger;
};

class FaceFinder
{
public:

    using FrameInput = ogles_gpgpu::FrameInput;
    
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

    FaceFinder(std::shared_ptr<drishti::face::FaceDetectorFactory> &factory, Config &config, void *glContext = nullptr);

    virtual GLuint operator()(const FrameInput &frame);
    
    void setMaxDistance(float meters);
    void setMinDistance(float meters);

protected:

    void createColormap(); // [0..359];

    void init2(drishti::face::FaceDetectorFactory &resources);
    void detect2(const FrameInput &frame, ScenePrimitives &scene);

    void init(const FrameInput &frame);
    void dump(std::vector<cv::Mat4b> &frames);
    virtual int detect(const FrameInput &frame, ScenePrimitives &scene);
    virtual void preprocess(const FrameInput &frame, ScenePrimitives &scene); // compute acf
    virtual GLuint paint(const ScenePrimitives &scene, GLuint inputTexture);

    void fill(drishti::acf::Detector::Pyramid &P);

    cv::Mat3f m_colors32FC3; // map angles to colors

    void *m_glContext = nullptr;
    bool m_hasInit = false;
    float m_scale = 2.0f;
    int m_outputOrientation = 0;

    uint64_t m_frameIndex = 0;

    using time_point = std::chrono::high_resolution_clock::time_point;
    std::pair<time_point, std::vector<cv::Rect>> m_objects;

    drishti::acf::Detector::Pyramid m_P;

    float m_minDistanceMeters = 0.f;
    float m_maxDistanceMeters = 10.0f;

    bool m_doLandmarks = false;
    int m_landmarksWidth = 256;

    bool m_doFlow = false;
    int m_flowWidth = 256;

    bool m_doFlash = false;
    int m_flashWidth = 128;

    std::shared_ptr<ogles_gpgpu::FifoProc> m_fifo;
    std::shared_ptr<ogles_gpgpu::ACF> m_acf;

    drishti::acf::Detector *m_detector = nullptr; // weak ref

    std::shared_ptr<drishti::face::FaceDetector> m_faceDetector;
    std::shared_ptr<ogles_gpgpu::FacePainter> m_painter;
    std::shared_ptr<ogles_gpgpu::TransformProc> m_rotater; // For QT
    std::shared_ptr<ogles_gpgpu::FlashFilter> m_flasher; // EXPERIMENTAL

    std::shared_ptr<ogles_gpgpu::EyeFilter> m_eyeFilter;

#if DRISHTI_FACEFILTER_DO_ELLIPSO_POLAR
    std::shared_ptr<ogles_gpgpu::EllipsoPolarWarp> m_ellipsoPolar[2];
#endif

    int m_index = 0;
    std::vector<std::future<ScenePrimitives>> m_scenes;

    TimerInfo m_timerInfo;
    
    std::shared_ptr<drishti::face::FaceDetectorFactory> m_factory;
    std::shared_ptr<drishti::sensor::SensorModel> m_sensor;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<ThreadPool<128>> m_threads;
};

#endif // FACE_FINDER_H
