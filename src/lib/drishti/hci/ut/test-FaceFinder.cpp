/*!
  @file   test-FaceFinder.cpp
  @author David Hirvonen
  @brief  High level FaceFinder test.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// https://code.google.com/p/googletest/wiki/Primer

#define DRISHTI_HCI_TEST_WARM_UP_GPU 0 // for timing only
#define DRISHTI_HCI_TEST_DISPLAY_OUTPUT 0

const char *sFaceDetector;
const char *sFaceDetectorMean;
const char *sFaceRegressor;
const char *sEyeRegressor;
const char *sImageFilename;

#if DRISHTI_HCI_DO_GPU
#  include "drishti/qtplus/QGLContext.h"
#endif

#if DRISHTI_SERIALIZE_WITH_CEREAL
#  include "drishti/core/drishti_stdlib_string.h"
#  include <cereal/archives/portable_binary.hpp>
#  include <cereal/types/vector.hpp>
#endif

#if DRISHTI_SERIALIZE_WITH_BOOST
#  include "drishti/core/drishti_serialization_boost.h"
#endif

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
#  define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#  define DFLT_TEXTURE_FORMAT GL_BGRA
#endif

#include <iostream>
#include <chrono>

#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }

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

        // Load the ground truth data:
        image = loadImage(sImageFilename);
        
        // TODO: we need to load ground truth output for each shader
        // (some combinations could be tested, but that is probably excessive!)
        //truth = loadImage(truthFilename);
        
        // Create FaceDetectorFactory (default file based):
        m_factory = std::make_shared<drishti::face::FaceDetectorFactory>();
        m_factory->sFaceDetector = sFaceDetector;
        m_factory->sFaceRegressors = { sFaceRegressor };
        m_factory->sEyeRegressor = sEyeRegressor;
        m_factory->sFaceDetectorMean = sFaceDetectorMean;
        
        // Create configuration:
        m_config.logger = drishti::core::Logger::create("test-drishti-hci");
        m_config.outputOrientation = 0;
        m_config.frameDelay = 2;
        m_config.doLandmarks = true;
        m_config.doFlow = true;
        m_config.doFlash = true;
        
#if DRISHTI_HCI_DO_GPU
        m_context = std::make_shared<QGLContext>();
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
    std::shared_ptr<drishti::hci::FaceFinder>
    create(const cv::Size &size, int orientation, bool doThreads)
    {
        if(doThreads)
        {
            m_config.threads = std::make_shared<ThreadPool<128>>();
        }
        else
        {
            m_config.threads.reset();
        }
        
        {// Create a sensor specification
            const float fx = size.width;
            const cv::Point2f p(image.cols/2, image.rows/2);
            drishti::sensor::SensorModel::Intrinsic params(p, fx, size);
            m_config.sensor = std::make_shared<drishti::sensor::SensorModel>(params);
        }
        
        m_config.outputOrientation = orientation;
        
        auto detector = std::make_shared<drishti::hci::FaceFinder>(m_factory, m_config, m_glContext);
        detector->setMinDistance(0.0);
        detector->setMaxDistance(1.0);
        return detector;
    }
    
    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}
    
    static cv::Mat loadImage(const std::string &filename)
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
        
        ogles_gpgpu::FrameInput frame({image.cols, image.rows}, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        
        const int iterations = 10;
        for(int i = 0; i < iterations; i++)
        {
            (*detector)(frame);
            
            // Wait on face request callback:
            monitor.wait();
            if(monitor.isInitialized())
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
    void analyzeFaceRequest(const std::vector<drishti::hci::FaceMonitor::FaceImage> &images)
    {
        std::vector<cv::Mat> faces, eyes;
        for(auto &f : images)
        {
            if(!f.image.empty())
            {
                faces.push_back(f.image);
            }
            if(!f.eyes.empty())
            {
                eyes.push_back(f.eyes);
            }
        }
        
        if((eyes.size() == 3) && !images[0].extra.empty())
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
        
        if(faces.size() > 0)
        {
            cv::Mat canvas;
            cv::hconcat(faces, canvas);
            cv::resize(canvas, canvas, {512, canvas.rows * 512/canvas.cols});
            cv::imshow("faces", canvas);
        }
    }
#endif
    
    drishti::hci::FaceFinder::Config m_config;
    std::shared_ptr<drishti::face::FaceDetectorFactory> m_factory;
    
#if DRISHTI_HCI_DO_GPU
    std::shared_ptr<QGLContext> m_context;
#endif
    
    void * m_glContext = nullptr;
    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<drishti::hci::FaceFinder> m_detector;

    // Test images:
    cv::Mat image, truth;
};

#if DRISHTI_HCI_DO_GPU
TEST_F(HCITest, RunTestGPUAsync)
{
    static const bool doCpu = false;
    static const bool doAsync = true;
    runTest(doCpu, doAsync);
}

TEST_F(HCITest, RunTestCPUAsync)
{
    static const bool doCpu = true;
    static const bool doAsync = true;
    runTest(doCpu, doAsync);
}

#endif // DRISHTI_HCI_DO_GPU

END_EMPTY_NAMESPACE
