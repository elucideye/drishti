/*! -*-c++-*-
  @file   test-FaceFinder.cpp
  @author David Hirvonen
  @brief  High level FaceFinder test.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// https://code.google.com/p/googletest/wiki/Primer

#define DRISHTI_HCI_TEST_WARM_UP_GPU 0 // for timing only
#define DRISHTI_HCI_TEST_DISPLAY_OUTPUT 0

extern const char* sFaceDetector;
extern const char* sFaceDetectorMean;
extern const char* sFaceRegressor;
extern const char* sEyeRegressor;
extern const char* sFaceImageFilename; // sImageFilename);

// clang-format off
#if defined(DRISHTI_DO_GPU_TESTING)
#  include "aglet/GLContext.h"
#endif
// clang-format on

#include "drishti/core/drishti_stdlib_string.h"
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>

#include "drishti/hci/FaceFinder.h"
#include "drishti/sensor/Sensor.h"
#include "drishti/core/ThreadPool.h"
#include "drishti/core/Logger.h"

#include "FaceMonitorHCITest.h"
#include "test-hessian-cpu.h"

#include <gtest/gtest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>
#include <memory>
#include <condition_variable>

#ifdef ANDROID
#define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#define DFLT_TEXTURE_FORMAT GL_BGRA
#endif

#include <iostream>
#include <chrono>

// clang-format off
#define BEGIN_EMPTY_NAMESPACE  namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

BEGIN_EMPTY_NAMESPACE

struct WaitKey
{
    WaitKey() {}
    ~WaitKey()
    {
#if DRISHTI_HCI_TEST_DISPLAY_OUTPUT
        cv::waitKey(0);
#endif
    }
};

class HCITest : public ::testing::Test
{
protected:
    bool m_hasTranspose = false;

    // Setup
    HCITest()
    {
        m_logger = drishti::core::Logger::create("test-drishti-hci");
        m_logger->set_level(spdlog::level::off); // by default...

        // Load the ground truth data:
        image = loadImage(sFaceImageFilename);

        // TODO: we need to load ground truth output for each shader
        // (some combinations could be tested, but that is probably excessive!)
        //truth = loadImage(truthFilename);

        // Create FaceDetectorFactory (default file based):
        m_factory = std::make_shared<drishti::face::FaceDetectorFactory>();
        m_factory->sFaceDetector = sFaceDetector;
        m_factory->sFaceRegressor = sFaceRegressor;
        m_factory->sEyeRegressor = sEyeRegressor;
        m_factory->sFaceDetectorMean = sFaceDetectorMean;

        // Create configuration:
        m_settings.logger = drishti::core::Logger::create("test-drishti-hci");
        m_settings.outputOrientation = 0;
        m_settings.frameDelay = 2;
        m_settings.doLandmarks = true;
        m_settings.doFlow = false;
        m_settings.doBlobs = false;

        m_settings.doSingleFace = true;
        m_settings.minDetectionDistance = 0.1f;
        m_settings.maxDetectionDistance = 0.5f;
        m_settings.faceFinderInterval = 0.f;
        m_settings.acfCalibration = 0.001f;
        m_settings.regressorCropScale = 1.5;

        m_settings.minTrackHits = 0; // allow all detections for testing
        m_settings.maxTrackMisses = 1;
        m_settings.minFaceSeparation = 0.15f;
        
        m_settings.history = 3;

#if defined(DRISHTI_DO_GPU_TESTING)
        m_context = aglet::GLContext::create(aglet::GLContext::kAuto);
#endif
    }

    // Cleanup
    virtual ~HCITest()
    {
        drishti::core::Logger::drop("test-drishti-hci");
    }

    /*
     * FaceFinder configuration can be achieved by modifying the input
     * configurations stored as member variables in the test fixture prior
     * to calling create()
     * 
     * @param size  : size of input frames for detector
     * @orientation : orientation of input frames
     * @doThreads   : support testing with and without threadpool
     */
    std::unique_ptr<drishti::hci::FaceFinder>
    create(const cv::Size& size, int orientation, bool doThreads)
    {
        if (doThreads)
        {
            m_settings.threads = std::make_shared<tp::ThreadPool<>>();
        }
        else
        {
            m_settings.threads.reset();
        }

        { // Create a sensor specification
            const float fx = size.width;
            const cv::Point2f p(image.cols / 2, image.rows / 2);
            drishti::sensor::SensorModel::Intrinsic params(p, fx, size);
            m_settings.sensor = std::make_shared<drishti::sensor::SensorModel>(params);
        }

        m_settings.outputOrientation = orientation;

        auto detector = drishti::hci::FaceFinder::create(m_factory, m_settings, m_glContext);
        return detector;
    }

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    static cv::Mat loadImage(const std::string& filename)
    {
        assert(!filename.empty());
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);

        assert(!image.empty() && image.type() == CV_8UC3);
        cv::Mat tmp;
        cv::cvtColor(image, tmp, cv::COLOR_BGR2BGRA);
        cv::swap(image, tmp);
        return image;
    }

    void runTest(bool doCpu, bool doAsync)
    {
#if DRISHTI_HCI_TEST_DISPLAY_OUTPUT
        WaitKey waitKey;
#endif

        // Instantiate a face finder and register a callback:
        auto detector = create(image.size(), 0, doAsync);
        detector->setDoCpuAcf(doCpu);

        FaceMonitorHCITest monitor;
        detector->registerFaceMonitorCallback(&monitor);

        const int iterations = 10;
        for (int i = 0; i < iterations; i++)
        {
            cv::flip(image, image, 1);
            ogles_gpgpu::FrameInput frame({ image.cols, image.rows }, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
            
            (*detector)(frame);

            // Wait on face request callback:
            monitor.wait();
            if (monitor.isInitialized())
            {
                // Wait on FaceMonitor::isValid() event
                GTEST_ASSERT_GT(monitor.getFaces().size(), 0);
            }

#if DRISHTI_HCI_TEST_DISPLAY_OUTPUT
            analyzeFaceRequest(monitor.getFaces());
#endif // DRISHTI_HCI_TEST_DISPLAY_OUTPUT

            monitor.clear();
        }
    }

#if DRISHTI_HCI_TEST_DISPLAY_OUTPUT
    void analyzeFaceRequest(const std::vector<drishti::hci::FaceMonitor::FaceImage>& images)
    {
        std::vector<cv::Mat> faces, eyes;
        for (auto& f : images)
        {
            if (!f.image.empty())
            {
                faces.push_back(f.image);
            }
            if (!f.eyes.empty())
            {
                eyes.push_back(f.eyes);
            }
        }

        if ((eyes.size() == 3) && !images[0].extra.empty())
        {
            WaitKey waitKey;

            { // eye stack
                cv::Mat canvas;
                cv::vconcat(eyes, canvas);
                cv::imshow("eyes", canvas);
            }

            { // gpu results:
                cv::imshow("ogles_gpgpu_FlashFilter_det1", images[0].extra);
            }
        }

        if (faces.size() > 0)
        {
            cv::Mat canvas;
            cv::hconcat(faces, canvas);
            cv::resize(canvas, canvas, { 512, canvas.rows * 512 / canvas.cols });
            cv::imshow("faces", canvas);
        }
    }
#endif

    drishti::hci::FaceFinder::Settings m_settings;
    std::shared_ptr<drishti::face::FaceDetectorFactory> m_factory;

#if defined(DRISHTI_DO_GPU_TESTING)
    std::shared_ptr<aglet::GLContext> m_context;
#endif

    void* m_glContext = nullptr;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<drishti::hci::FaceFinder> m_detector;

    // Test images:
    cv::Mat image, truth;
};

#if defined(DRISHTI_DO_GPU_TESTING)
TEST_F(HCITest, RunTestCPUAsync)
{
    static const bool doCpu = true;
    static const bool doAsync = true;
    runTest(doCpu, doAsync);
}

TEST_F(HCITest, RunTestGPUAsync)
{
    static const bool doCpu = false;
    static const bool doAsync = true;
    runTest(doCpu, doAsync);
}
#endif // defined(DRISHTI_DO_GPU_TESTING)

END_EMPTY_NAMESPACE
