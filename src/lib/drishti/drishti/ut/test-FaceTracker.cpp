/*!
  @file   test-FaceTracker.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API FaceTracker interface.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/drishti/FaceTracker.hpp"
#include "drishti/drishti/Context.hpp"
#include "drishti/drishti/drishti_cv.hpp"
#include "drishti/core/Logger.h"
#include "drishti/core/ThreadPool.h"

// clang-format off
#if defined(DRISHTI_DRISHTI_DO_GPU)
#  include "aglet/GLContext.h"
#endif
// clang-format on

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <memory>
#include <fstream>

// clang-format off
#ifdef ANDROID
#  define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#  define DFLT_TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

const char* sFaceDetector;
const char* sFaceDetectorMean;
const char* sFaceRegressor;
const char* sEyeRegressor;
const char* sImageFilename;

int gauze_main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc == 6);
    sFaceDetector = argv[1];
    sFaceDetectorMean = argv[2];
    sFaceRegressor = argv[3];
    sEyeRegressor = argv[4];
    sImageFilename = argv[5];
    return RUN_ALL_TESTS();
}

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
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

class FaceTest : public ::testing::Test
{
protected:
    bool m_hasTranspose = false;

    // Setup
    FaceTest()
    {
        m_logger = drishti::core::Logger::create("test-drishti-drishti-face");
        m_logger->set_level(spdlog::level::off); // by default...

        // Load the ground truth data:
        image = loadImage(sImageFilename);

#if defined(DRISHTI_DRISHTI_DO_GPU)
        m_context = aglet::GLContext::create(aglet::GLContext::kAuto);
#endif

        // TODO: we need to load ground truth output for each shader
        // (some combinations could be tested, but that is probably excessive!)
        //truth = loadImage(truthFilename);
    }

    // Cleanup
    virtual ~FaceTest()
    {
        drishti::core::Logger::drop("test-drishti-drishti-face");
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
    std::shared_ptr<drishti::sdk::FaceTracker> create(const cv::Size& size, int orientation, bool doThreads)
    {
        const float fx = size.width;
        const drishti::sdk::Vec2f p(image.cols / 2, image.rows / 2);
        drishti::sdk::SensorModel::Intrinsic intrinsic(p, fx, {size.width, size.height});
        drishti::sdk::Matrix33f I = drishti::sdk::Matrix33f::eye();
        drishti::sdk::SensorModel::Extrinsic extrinsic(I);
        drishti::sdk::SensorModel sensor(intrinsic, extrinsic);

        drishti::sdk::Context context(sensor);

        /*
         * Lazy construction requires that streams are in scope at the time of construction:
         */

        std::ifstream iFaceDetector(sFaceDetector, std::ios_base::binary);
        if (!iFaceDetector)
        {
            throw std::runtime_error("FaceTest::create() failed to open face detector");
        }

        std::ifstream iFaceRegressor(sFaceRegressor, std::ios_base::binary);
        if (!iFaceRegressor)
        {
            throw std::runtime_error("FaceTest::create() failed to open face regressor");
        }

        std::ifstream iEyeRegressor(sEyeRegressor, std::ios_base::binary);
        if (!iEyeRegressor)
        {
            throw std::runtime_error("FaceTest::create() failed to open eye regressor");
        }

        std::ifstream iFaceDetectorMean(sFaceDetectorMean, std::ios_base::binary);
        if (!iFaceDetectorMean)
        {
            throw std::runtime_error("FaceTest::create() failed to open face mean");
        }

        assert(iFaceDetector.good());
        assert(iFaceRegressor.good());
        assert(iEyeRegressor.good());
        assert(iFaceDetectorMean.good());

        drishti::sdk::FaceTracker::Resources factory;
        factory.sFaceDetector = &iFaceDetector;
        factory.sFaceRegressor = &iFaceRegressor;
        factory.sEyeRegressor = &iEyeRegressor;
        factory.sFaceModel = &iFaceDetectorMean;

        auto tracker = std::make_shared<drishti::sdk::FaceTracker>(&context, factory);

        return tracker;
    }

    void runTest(bool doCpu, bool doAsync)
    {
        // Instantiate a face finder and register a callback:
        auto tracker = create(image.size(), 0, doAsync);

        ASSERT_TRUE(tracker->good());

        drishti::sdk::VideoFrame frame({ image.cols, image.rows }, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

        const int iterations = 10;
        for (int i = 0; i < iterations; i++)
        {
            (*tracker)(frame);
        }
    }

#ifdef DRISHTI_BUILD_C_INTERFACE
    std::shared_ptr<drishti::sdk::FaceTracker> createC(const cv::Size& size, int orientation, bool doThreads)
    {
        const float fx = size.width;
        const cv::Point2f p(image.cols / 2, image.rows / 2);
        drishti::sensor::SensorModel::Intrinsic params(p, fx, size);
        drishti::sensor::SensorModel sensor(params);

        drishti::sdk::Context context(sensor);
        context.setMinDetectionDistance(0.0);
        context.setMaxDetectionDistance(1.0);

        std::ifstream iFaceDetector(sFaceDetector, std::ios_base::binary);
        std::ifstream iFaceRegressor(sFaceRegressor, std::ios_base::binary);
        std::ifstream iEyeRegressor(sEyeRegressor, std::ios_base::binary);
        std::ifstream iFaceDetectorMean(sFaceDetectorMean, std::ios_base::binary);

        drishti::sdk::FaceTracker::Resources factory;
        factory.sFaceDetector = &iFaceDetector;
        factory.sFaceRegressors = { &iFaceRegressor };
        factory.sEyeRegressor = &iEyeRegressor;
        factory.sFaceModel = &iFaceDetectorMean;

        auto tracker = drishti_face_tracker_create_from_streams(&context, factory);

        if (tracker)
        {
            if (tracker->good())
            {
                return std::shared_ptr<drishti::sdk::FaceTracker>(tracker, drishti_face_tracker_destroy);
            }
            else
            {
                delete tracker;
            }
            return nullptr;
        }
        else
        {
            m_logger->error("Unable to instantiate face tracker");
            return nullptr;
        }
    }

    int callback(drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results)
    {
        m_logger->info("callback: Received results");

        int count = 0;
        for (const auto& r : results)
        {
            //std::stringstream ss;
            //ss << "/tmp/image_" << count++ << ".png";
            //cv::imwrite(ss.str(), drishti::sdk::drishtiToCv<drishti::sdk::Vec4b, cv::Vec4b>(r.image));
        }

        return 0;
    }

    int trigger(const drishti::sdk::Vec3f& point, double timestamp)
    {
        m_logger->info("trigger: Received results: {}, {}, {} {}", point[0], point[1], point[2], timestamp);
        return 1; // force trigger
    }

    int allocator(const drishti_image_t& spec, drishti::sdk::Image4b& image)
    {
        m_logger->info("allocator: {} {}", spec.width, spec.height);
        return 0;
    }

    /*
     * C API:
     */

    static int callbackFunc(void* context, drishti::sdk::Array<drishti_face_tracker_result_t, 64>& results)
    {
        if (FaceTest* ft = static_cast<FaceTest*>(context))
        {
            return ft->callback(results);
        }
        return -1;
    }

    static int triggerFunc(void* context, const drishti::sdk::Vec3f& point, double timestamp)
    {
        if (FaceTest* ft = static_cast<FaceTest*>(context))
        {
            return ft->trigger(point, timestamp);
        }
        return -1;
    }

    static int allocatorFunc(void* context, const drishti_image_t& spec, drishti::sdk::Image4b& image)
    {
        if (FaceTest* ft = static_cast<FaceTest*>(context))
        {
            return ft->allocator(spec, image);
        }
        return -1;
    }

    void runTestC(bool doCpu, bool doAsync)
    {
        // Instantiate a face finder and register a callback:
        auto tracker = createC(image.size(), 0, doAsync);

        drishti_face_tracker_t table{
            this,
            triggerFunc,
            callbackFunc,
            allocatorFunc
        };

        drishti_face_tracker_callback(tracker.get(), table);

        drishti::sdk::VideoFrame frame({ image.cols, image.rows }, image.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

        const int iterations = 10;
        for (int i = 0; i < iterations; i++)
        {
            drishti_face_tracker_track(tracker.get(), frame);
        }
    }
#endif // DRISHTI_BUILD_C_INTERFACE

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

#if defined(DRISHTI_DRISHTI_DO_GPU)
    std::shared_ptr<aglet::GLContext> m_context;
#endif

    void* m_glContext = nullptr;
    std::shared_ptr<spdlog::logger> m_logger;

    // Test images:
    cv::Mat image, truth;
};

TEST_F(FaceTest, RunSimpleTest)
{
    static const bool doCpu = false;
    static const bool doAsync = true;
    runTest(doCpu, doAsync);
}

#ifdef DRISHTI_BUILD_C_INTERFACE
TEST_F(FaceTest, RunSimpleTestC)
{
    static const bool doCpu = true;
    static const bool doAsync = true;
    runTestC(doCpu, doAsync);
}

#endif // DRISHTI_BUILD_C_INTERFACE

END_EMPTY_NAMESPACE
