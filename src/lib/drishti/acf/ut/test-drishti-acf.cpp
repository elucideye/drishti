/*! -*-c++-*-
  @file   test-drishti-acf.cpp
  @author David Hirvonen
  @brief  CPU ACF shader tests using a google test fixture.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file has various tests for comparing GPU ACF output with the
  reference CPU ACF output.  This is a WIP and there is currently 
  liberal use of cv::imshow() for visualization, etc.  This needs to
  be automated and reasonable tolerances on GPU vs CPU discrepancies 
  need to be established.

*/

// https://code.google.com/p/googletest/wiki/Primer

#define DRISHTI_ACF_TEST_DISPLAY_OUTPUT 0
#define DRISHTI_ACF_TEST_WARM_UP_GPU 0 // for timing only

// clang-format off
#if defined(DRISHTI_DO_GPU_TESTING)
#  include "drishti/acf/GPUACF.h"
#  include "aglet/GLContext.h"
#endif
// clang-format on

#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"

// http://uscilab.github.io/cereal/serialization_archives.html
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>

// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!# Work in progress !#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!
// #!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!#!

#include <gtest/gtest.h>

#include "drishti/core/drawing.h"
#include "drishti/acf/ACF.h"
#include "drishti/acf/MatP.h"
#include "drishti/core/Logger.h"
#include "drishti/geometry/Primitives.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <fstream>
#include <memory>

//const char* imageFilename; -> sFaceImageFilename
//const char* modelFilename; -> sFaceDetector

extern const char* sFaceImageFilename;
extern const char* sFaceDetector;

// clang-format off
#ifdef ANDROID
#  define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#  define DFLT_TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

#include <iostream>
#include <chrono>

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

BEGIN_EMPTY_NAMESPACE

struct WaitKey
{
    WaitKey() {}
    ~WaitKey()
    {
#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
        cv::waitKey(0);
#endif
    }
};

// http://stackoverflow.com/a/32647694
static bool isEqual(const cv::Mat& a, const cv::Mat& b);
static bool isEqual(const drishti::acf::Detector& a, const drishti::acf::Detector& b);
static cv::Mat draw(drishti::acf::Detector::Pyramid& pyramid);

class ACFTest : public ::testing::Test
{
protected:
    bool m_hasTranspose = false;

    // Setup
    ACFTest()
    {
        m_logger = drishti::core::Logger::create("test-drishti-acf");
        m_logger->set_level(spdlog::level::off); // by default...

        // Load the ground truth data:
        image = loadImage(sFaceImageFilename);
        if (m_hasTranspose)
        {
            image = image.t();
        }

        // Load input suitable for ACF processing
        // * single precision floatin point
        // * RGB channel order
        loadACFInput(sFaceImageFilename);

#if defined(DRISHTI_DO_GPU_TESTING)
        m_context = aglet::GLContext::create(aglet::GLContext::kAuto);
        CV_Assert(m_context && (*m_context));
#endif
    }

    // Cleanup
    virtual ~ACFTest()
    {
        drishti::core::Logger::drop("test-drishti-acf");
    }

    std::shared_ptr<drishti::acf::Detector> create(const std::string& filename, bool do10Channel = true)
    {
        return std::make_shared<drishti::acf::Detector>(filename);
    }

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    std::shared_ptr<drishti::acf::Detector> getDetector()
    {
        if (!m_detector)
        {
            m_detector = create(sFaceDetector);
        }
        return m_detector;
    }

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

    // Load and format a single image for reuse in multiple tests:
    void loadACFInput(const std::string& filename)
    {
        cv::Mat I = loadImage(filename);
        cv::cvtColor(I, m_I, cv::COLOR_BGR2RGB);
        m_I.convertTo(m_I, CV_32FC3, (1.0 / 255.0));
        m_IpT = MatP(m_I.t());
        m_hasTranspose = false;
    }

#if defined(DRISHTI_DO_GPU_TESTING)

    static std::vector<ogles_gpgpu::Size2d> getPyramidSizes(drishti::acf::Detector::Pyramid& Pcpu)
    {
        std::vector<ogles_gpgpu::Size2d> sizes;
        for (int i = 0; i < Pcpu.nScales; i++)
        {
            const auto size = Pcpu.data[i][0][0].size();
            sizes.emplace_back(size.height * 4, size.width * 4); // transpose
        }
        return sizes;
    }

    // Utility method for code reuse in common GPU tests:
    //
    // Output:
    // 1) drishti::acf::Detector::Pyramid from GPU
    //
    // State:
    // 1) Allocates drishti::acf::Detector
    // 2) Allocates ogles_gpgpu::ACF

    void initGPUAndCreatePyramid(drishti::acf::Detector::Pyramid& Pgpu)
    {
        m_detector = create(sFaceDetector);

        // Compute a reference pyramid on the CPU:
        drishti::acf::Detector::Pyramid Pcpu;
        m_detector->computePyramid(m_IpT, Pcpu);

        ASSERT_NE(m_detector, nullptr);

        m_detector->setIsTranspose(true);
        m_detector->computePyramid(m_IpT, Pcpu);
        auto sizes = getPyramidSizes(Pcpu);
        static const bool doGrayscale = false;
        static const bool doCorners = false;
        ogles_gpgpu::Size2d inputSize(image.cols, image.rows);

        m_acf = std::make_shared<ogles_gpgpu::ACF>(nullptr, inputSize, sizes, ogles_gpgpu::ACF::kLUVM012345, doGrayscale, doCorners, false);
        m_acf->setRotation(0);

        cv::Mat input = image;
#if DRISHTI_ACF_TEST_WARM_UP_GPU
        for (int i = 0; i < 10; i++)
        {
            (*m_acf)({ input.cols, input.rows }, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        }
#endif

        (*m_acf)({ input.cols, input.rows }, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);
        m_acf->fill(Pgpu, Pcpu);

        {
            // This code block is a place holder for 7 channel ACF output, which conveniently fits in 2 textures
            // for faster transfers on low performing Android devices.  Currently there is no 7 channel ACF
            // classifier/detector, so this is used as a place holder to illustrate the raw channel extraction
            // and pyramid formatting until equivalent CPU formatting is in place.
            auto acf7 = std::make_shared<ogles_gpgpu::ACF>(nullptr, inputSize, sizes, ogles_gpgpu::ACF::kM012345, doGrayscale, doCorners, false);
            (*acf7)({ input.cols, input.rows }, input.ptr(), true, 0, DFLT_TEXTURE_FORMAT);

            drishti::acf::Detector::Pyramid Pgpu7;
            acf7->fill(Pgpu7, Pcpu);

            //cv::imshow("Pgpu7", draw(Pgpu7);
            //cv::imshow("LM012345", acf7->getChannels());
            //cv::imshow("LUVM012345", m_acf->getChannels());
            //cv::waitKey(0);
        }
    }

    std::shared_ptr<aglet::GLContext> m_context;
    std::shared_ptr<ogles_gpgpu::ACF> m_acf;
#endif

    std::shared_ptr<spdlog::logger> m_logger;

    std::shared_ptr<drishti::acf::Detector> m_detector;

    // Test images:
    cv::Mat image, truth;

    // ACF inputs: RGB single precision floating point
    cv::Mat m_I;
    MatP m_IpT;
};

#if defined(DRISHTI_DO_GPU_TESTING)
static cv::Mat getImage(ogles_gpgpu::ProcInterface& proc)
{
    cv::Mat result(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4);
    proc.getResultData(result.ptr());
    return result;
}
#endif // defined(DRISHTI_DO_GPU_TESTING)

// This is a WIP, currently we test the basic CPU detection functionality
// with a sample image.  Given the complexity of the GPU implementation,
// more tests will need to be added for lower level channel computation,
// such as CPU vs GPU error bounds.  For now the simple detection success
// test and a simple placeholder assert(true) test wil be added at the
// end of the test.

static void draw(cv::Mat& canvas, const std::vector<cv::Rect>& objects)
{
    for (const auto& r : objects)
    {
        cv::rectangle(canvas, r, { 0, 255, 0 }, 1, 8);
    }
}

TEST_F(ACFTest, ACFDetectionCPUMat)
{
    WaitKey waitKey;

    auto detector = getDetector();
    ASSERT_NE(detector, nullptr);

    std::vector<double> scores;
    std::vector<cv::Rect> objects;
    detector->setIsTranspose(false);
    (*detector)(m_I, objects, &scores);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    cv::Mat canvas = image.clone();
    draw(canvas, objects);
    cv::imshow("acf_cpu_detection_mat", canvas);
#endif

    ASSERT_GT(objects.size(), 0); // Very weak test!!!
}

TEST_F(ACFTest, ACFDetectionCPUMatP)
{
    WaitKey waitKey;

    auto detector = getDetector();
    ASSERT_NE(detector, nullptr);

    std::vector<double> scores;
    std::vector<cv::Rect> objects;

    // Input is transposed, but objects will be returned in upright coordinate system
    detector->setIsTranspose(true);
    (*detector)(m_IpT, objects, &scores);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    cv::Mat canvas = image.clone();
    draw(canvas, objects);
    cv::imshow("acf_cpu_detection_matp", canvas);
#endif

    ASSERT_GT(objects.size(), 0); // Very weak test!!!
}

// Pull out the ACF intermediate results from the logger:
//
//using ChannelLogger = int(const cv::Mat &, const std::string &);
//cv::Mat M, Mnorm, O, L, U, V, H;
//std::function<ChannelLogger> logger = [&](const cv::Mat &I, const std::string &tag) -> int
//{
//    if(tag.find("L:") != std::string::npos) { L = I.t(); }
//    else if(tag.find("U:") != std::string::npos) { U = I.t(); }
//    else if(tag.find("V:") != std::string::npos) { V = I.t(); }
//    else if(tag.find("Mnorm:") != std::string::npos) { Mnorm = I.t(); }
//    else if(tag.find("M:") != std::string::npos) { M = I.t(); }
//    else if(tag.find("O:") != std::string::npos) { O = I.t(); }
//    else if(tag.find("H:") != std::string::npos) { H = I.t(); }
//    return 0;
//};
//detector->setLogger(logger);

TEST_F(ACFTest, ACFChannelsCPU)
{
    auto detector = getDetector();
    ASSERT_NE(detector, nullptr);

    MatP Ich;
    detector->setIsTranspose(true);
    detector->computeChannels(m_IpT, Ich);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    cv::Mat canvas = Ich.base().t();
    WaitKey waitKey;
    cv::imshow("acf_channel_cpu", canvas);
#endif

    // TODO: comparison for channels:
    // load cereal pba cv::Mat, compare precision, etc
    ASSERT_EQ(Ich.base().empty(), false);
}

// NOTE: side-effect, set's the CPU pyramid for GPU tests:
TEST_F(ACFTest, ACFPyramidCPU)
{
    auto detector = getDetector();
    ASSERT_NE(detector, nullptr);

    auto pyramid = std::make_shared<drishti::acf::Detector::Pyramid>();
    detector->setIsTranspose(true);
    detector->computePyramid(m_IpT, *pyramid);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    WaitKey waitKey;
    cv::Mat canvas = draw(*pyramid);
    cv::imshow("acf_pyramid_cpu", canvas.t());
#endif

    // TODO: comparision for pyramid:
    // load cereal pba cv::Mat, compare precision, etc
    ASSERT_GT(pyramid->data.max_size(), 0);
}

#if defined(DRISHTI_DO_GPU_TESTING)
TEST_F(ACFTest, ACFPyramidGPU10)
{
    drishti::acf::Detector::Pyramid Pgpu;
    initGPUAndCreatePyramid(Pgpu);
    ASSERT_NE(m_detector, nullptr);
    ASSERT_NE(m_acf, nullptr);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    WaitKey waitKey;
    cv::Mat channels = m_acf->getChannels();
    cv::imshow("acf_gpu", channels);
#endif

    // TODO: comparision for pyramid:
    // load cereal pba cv::Mat, compare precision, etc
    ASSERT_GT(Pgpu.data.max_size(), 0);

    // Compare precision with CPU implementation (very loose)
}

TEST_F(ACFTest, ACFDetectionGPU10)
{
    drishti::acf::Detector::Pyramid Pgpu;
    initGPUAndCreatePyramid(Pgpu);
    ASSERT_NE(m_detector, nullptr);
    ASSERT_NE(m_acf, nullptr);

    std::vector<double> scores;
    std::vector<cv::Rect> objects;
    (*m_detector)(Pgpu, objects);

#if DRISHTI_ACF_TEST_DISPLAY_OUTPUT
    WaitKey waitKey;
    cv::Mat canvas = image.clone();
    draw(canvas, objects);
    cv::imshow("acf_gpu_detections", canvas);
#endif

    ASSERT_GT(objects.size(), 0); // Very weak test!!!
}
#endif // defined(DRISHTI_DO_GPU_TESTING)

// ### utility ###

// http://stackoverflow.com/a/32647694
static bool isEqual(const cv::Mat& a, const cv::Mat& b)
{
    cv::Mat temp;
    cv::bitwise_xor(a, b, temp); //It vectorizes well with SSE/NEON
    return !(cv::countNonZero(temp));
}

static bool isEqual(const drishti::acf::Detector& a, const drishti::acf::Detector& b)
{
    if (!isEqual(a.clf.fids, b.clf.fids))
    {
        std::cout << cv::Mat1b(a.clf.fids == b.clf.fids) << std::endl;

        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
        return false;
    }

    if (!isEqual(a.clf.child, b.clf.child))
    {
        std::cout << cv::Mat1b(a.clf.child == b.clf.child) << std::endl;

        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
        return false;
    }

    if (!isEqual(a.clf.depth, b.clf.depth))
    {
        std::cout << cv::Mat1b(a.clf.depth == b.clf.depth) << std::endl;

        cv::Mat tmp;
        cv::hconcat(a.clf.fids, b.clf.fids, tmp);
        std::cout << tmp << std::endl;
        return false;
    }

    return true;

    // The float -> uint16_t -> float will not be an exact match
    //isEqual(a.clf.thrs, b.clf.thrs) &&
    //isEqual(a.clf.hs, b.clf.hs) &&
    //isEqual(a.clf.weights, b.clf.weights) &&
}

static cv::Mat draw(drishti::acf::Detector::Pyramid& pyramid)
{
    cv::Mat canvas;
    std::vector<cv::Mat> levels;
    for (int i = 0; i < pyramid.nScales; i++)
    {
        // Concatenate the transposed faces, so they are compatible with the GPU layout
        cv::Mat Ccpu;
        std::vector<cv::Mat> images;
        for (const auto& image : pyramid.data[i][0].get())
        {
            images.push_back(image.t());
        }
        cv::vconcat(images, Ccpu);

        // Instead of upright:
        //cv::vconcat(pyramid.data[i][0].get(), Ccpu);

        if (levels.size())
        {
            cv::copyMakeBorder(Ccpu, Ccpu, 0, levels.front().rows - Ccpu.rows, 0, 0, cv::BORDER_CONSTANT);
        }

        levels.push_back(Ccpu);
    }
    cv::hconcat(levels, canvas);
    return canvas;
}

END_EMPTY_NAMESPACE
