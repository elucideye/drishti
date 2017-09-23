/*! -*-c++-*-
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
#include "drishti/acf/ACF.h" // needed for pyramid
#include "drishti/face/Face.h"
#include "drishti/face/FaceDetectorFactory.h"
#include "drishti/sensor/Sensor.h"
#include "thread_pool/thread_pool.hpp"

#include <memory>

#define DRISHTI_HCI_FACEFINDER_MIN_DISTANCE 0.1
#define DRISHTI_HCI_FACEFINDER_MAX_DISTANCE 0.4
#define DRISHTI_HCI_FACEFINDER_MIN_TRACK_HITS 3
#define DRISHTI_HCI_FACEFINDER_MAX_TRACK_MISSES 3
#define DRISHTI_HCI_FACEFINDER_MIN_SEPARATION 0.15f // meters
#define DRISHTI_HCI_FACEFINDER_INTERVAL 0.1f
#define DRISHTI_HCI_FACEFINDER_DO_ELLIPSO_POLAR 0
#define DRISHTI_HCI_FACEFINDER_HISTORY 3

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceFinder
{
public:
    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point; // <std::chrono::system_clock>;
    using FrameInput = ogles_gpgpu::FrameInput;
    using FeaturePoints = std::vector<FeaturePoint>;
    using ImageLogger = std::function<void(const cv::Mat& image)>;
    using FaceDetectorFactoryPtr = std::shared_ptr<drishti::face::FaceDetectorFactory>;

    struct TimerInfo
    {
        double detectionTime = 0.0;
        double regressionTime = 0.0;
        double eyeRegressionTime = 0.0;
        double acfProcessingTime = 0.0;
        double blobExtractionTime = 0.0;
        double renderSceneTime = 0.0;
        std::function<void(double second)> detectionTimeLogger;
        std::function<void(double second)> regressionTimeLogger;
        std::function<void(double second)> eyeRegressionTimeLogger;
        std::function<void(double second)> acfProcessingTimeLogger;
        std::function<void(double second)> blobExtractionTimeLogger;
        std::function<void(double second)> renderSceneTimeLogger;
        
        void init();

        friend std::ostream& operator<<(std::ostream& stream, const TimerInfo& info);
    };

    struct Settings
    {
        std::shared_ptr<drishti::sensor::SensorModel> sensor;
        std::shared_ptr<spdlog::logger> logger;
        std::shared_ptr<tp::ThreadPool<>> threads;
        ImageLogger imageLogger;
        int outputOrientation = 0;
        int frameDelay = 1;
        bool doLandmarks = true;
        bool doFlow = true;
        bool doBlobs = false;

        // Detection parameters:
        bool doSingleFace = false;
        float minDetectionDistance = DRISHTI_HCI_FACEFINDER_MIN_DISTANCE;
        float maxDetectionDistance = DRISHTI_HCI_FACEFINDER_MAX_DISTANCE;
        float faceFinderInterval = DRISHTI_HCI_FACEFINDER_INTERVAL;
        float acfCalibration = 0.f;
        float regressorCropScale = 0.f;

        // Detection tracks:
        std::size_t minTrackHits = DRISHTI_HCI_FACEFINDER_MIN_TRACK_HITS;
        std::size_t maxTrackMisses = DRISHTI_HCI_FACEFINDER_MAX_TRACK_MISSES;
        float minFaceSeparation = DRISHTI_HCI_FACEFINDER_MIN_SEPARATION;

        // OpengL parameters:
        int glVersionMajor = 2;
        int glVersionMinor = 0; // future use
        bool usePBO = false;
        bool doOptimizedPipeline = true;

        // Display parameters:
        bool renderFaces = true;
        bool renderPupils = true;
        bool renderCorners = true;
        
        int history = DRISHTI_HCI_FACEFINDER_HISTORY;
    };

    FaceFinder(FaceDetectorFactoryPtr& factory, Settings& config, void* glContext = nullptr);
    ~FaceFinder();

    static void tryEnablePlatformOptimizations();

    virtual void initialize(); // must call at startup

    static std::unique_ptr<FaceFinder>
    create(FaceDetectorFactoryPtr& factory, Settings& config, void* glContext = nullptr);

    virtual GLuint operator()(const FrameInput& frame);
    
    float getMaxDistance() const;
    float getMinDistance() const;

    void setDoCpuAcf(bool flag);
    bool getDoCpuAcf() const;

    void setFaceFinderInterval(double interval);
    double getFaceFinderInterval() const;

    void setBrightness(float value);

    void registerFaceMonitorCallback(FaceMonitor* callback);

    virtual bool doAnnotations() const;

    void setImageLogger(const ImageLogger& logger);

protected:
    
    using ImageViews = std::vector<core::ImageView>;
    using EyeModelPair = std::array<eye::EyeModel, 2>;
    using EyeModelPairs = std::vector<EyeModelPair>;
    
    std::pair<GLuint, ScenePrimitives> runFast(const FrameInput& frame, bool doDetection);
    std::pair<GLuint, ScenePrimitives> runSimple(const FrameInput& frame, bool doDetection);
    
    bool needsDetection(const TimePoint& ts) const;

    void computeGazePoints();
    void updateEyes(GLuint inputTexId, const ScenePrimitives& scene);

    void scaleToFullResolution(std::vector<drishti::face::FaceModel> &faces);
    
    void notifyListeners(const ScenePrimitives& scene, const TimePoint& time, bool isFull);

    virtual void init(const cv::Size& inputSize);
    virtual void initPainter(const cv::Size& inputSizeUp);
    void initFaceFilters(const cv::Size &inputSizeUp);
    void initACF(const cv::Size& inputSizeUp);
    void initFIFO(const cv::Size& inputSize, std::size_t n);
    void initBlobFilter();
    void initColormap(); // [0..359];
    void initEyeEnhancer(const cv::Size& inputSizeUp, const cv::Size& eyesSize);
    void initIris(const cv::Size& size);
    void initTimeLoggers();
    void init2(drishti::face::FaceDetectorFactory& resources);

    void dumpEyes(ImageViews& frames, EyeModelPairs& eyes, int n=1, bool getImage = false);
    void dumpFaces(ImageViews& frames, int n=1, bool getImage = false);
    int detectOnly(ScenePrimitives& scene, bool doDetection);
    virtual int detect(const FrameInput& frame, ScenePrimitives& scene, bool doDetection);
    virtual GLuint paint(const ScenePrimitives& scene, GLuint inputTexture);
    virtual void preprocess(const FrameInput& frame, ScenePrimitives& scene, bool needsDetection); // compute acf

    GLuint stabilize(GLuint inputTexId, const cv::Size &inputSizeUp, const drishti::face::FaceModel &face);
    int computeDetectionWidth(const cv::Size& inputSizeUp) const;
    void computeAcf(const FrameInput& frame, bool doLuv, bool doDetection);
    std::shared_ptr<acf::Detector::Pyramid> createAcfGpu(const FrameInput& frame, bool doDetection);
    std::shared_ptr<acf::Detector::Pyramid> createAcfCpu(const FrameInput& frame, bool doDetection);
    void fill(drishti::acf::Detector::Pyramid& P);

    struct Impl;
    std::unique_ptr<Impl> impl;
};

std::ostream& operator<<(std::ostream& stream, const FaceFinder::TimerInfo& info);

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceFinder_h__
