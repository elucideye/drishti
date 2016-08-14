/*!
  @file   finder/FaceFinder.h
  @author David Hirvonen
  @brief  Scene viewed by the camera represented by low level primitives: (corners, face, flow, etc.)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/
#ifndef FACE_FINDER_H
#define FACE_FINDER_H

#include "acf/GPUACF.h"
#include "acf/ACF.h"
#include "face/Face.h"
#include "sensor/Sensor.h"

#include "Scene.hpp"

#include "ogles_gpgpu/common/proc/flow.h"

#if USE_LOCAL_THREAD_POOL
#  include <thread_pool.hpp>
#else
#  include <thread-pool-cpp/thread_pool.hpp>
#endif

#include <memory>
#include <chrono>

namespace ogles_gpgpu
{
class FlashFilter;
class FacePainter;
class FifoProc;
class TransformProc;
class EyeFilter;
class EllipsoPolarWarp;
}

namespace spdlog
{
class logger;
}

namespace drishti
{
namespace face
{
class FaceModelEstimator;
class FaceDetector;
}
}

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

    FaceFinder(void *glContext, int outputOrientation, int delay=1);

    virtual GLuint operator()(const FrameInput &frame);

protected:

    void createColormap(); // [0..359];

    void init2();
    void detect2(const FrameInput &frame, ScenePrimitives &scene);

    void init(const FrameInput &frame);
    void dump(std::vector<cv::Mat4b> &frames);
    virtual int detect(const FrameInput &frame, ScenePrimitives &scene);
    virtual void preprocess(const FrameInput &frame, ScenePrimitives &scene); // compute acf
    virtual GLuint paint(const ScenePrimitives &scene, GLuint inputTexture);

    void fill(drishti::acf::Detector::Pyramid &P);

    cv::Mat3f m_colors32FC3; // map angles to colors

    std::shared_ptr<spdlog::logger> m_logger;
    //std::shared_ptr<drishti::core::Logger> m_logger;

    void *m_glContext = nullptr;
    bool m_hasInit = false;
    float m_scale = 2.0f;

    uint64_t m_frameIndex = 0;
    std::unique_ptr<ThreadPool<128>> m_threads;

    using time_point = std::chrono::high_resolution_clock::time_point;
    std::pair<time_point, std::vector<cv::Rect>> m_objects;

    drishti::sensor::SensorModel m_sensor;

    drishti::acf::Detector::Pyramid m_P;

    int m_regressionWidth = 256;
    bool m_doRegression = false;

    int m_cornerWidth = 256;
    bool m_doCorners = false;

    int m_flowWidth = 256;
    bool m_doFlow = false;

    int m_flashWidth = 128;
    bool m_doFlash = false;

    std::shared_ptr<ogles_gpgpu::FifoProc> m_fifo;
    std::shared_ptr<ogles_gpgpu::ACF> m_acf;

    drishti::acf::Detector *m_detector = nullptr; // weak ref

    std::shared_ptr<drishti::face::FaceDetector> m_faceDetector;
    std::shared_ptr<ogles_gpgpu::FacePainter> m_painter;
    std::shared_ptr<ogles_gpgpu::TransformProc> m_rotater; // For QT
    std::shared_ptr<ogles_gpgpu::FlashFilter> m_flasher; // EXPERIMENTAL

    std::shared_ptr<ogles_gpgpu::EyeFilter> m_eyeFilter;

    std::shared_ptr<ogles_gpgpu::EllipsoPolarWarp> m_ellipsoPolar[2];

    int m_outputOrientation = 0;

    int m_index = 0;
    std::vector< std::future<ScenePrimitives> > m_scenes;

    TimerInfo m_timerInfo;

};

#endif // FACE_FINDER_H
