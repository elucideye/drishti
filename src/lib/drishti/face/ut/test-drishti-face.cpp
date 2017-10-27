/*! -*-c++-*-
  @file   test-drishti-face.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Note that drishti::face:FaceFinder has a dependency on an active OpenGL rendering context,
  which is used (at the least) for creating upright rescaled video frames for ACF channel 
  computation, or (at the most) full ACF channel computation on the GPU using shaders.

*/

#include "drishti/face/FaceDetectorAndTracker.h"
#include "drishti/core/Logger.h"

#include <gtest/gtest.h>

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

extern const char* sFaceDetector;
extern const char* sFaceDetectorMean;
extern const char* sFaceRegressor;
extern const char* sEyeRegressor;
extern const char* sFaceImageFilename;

BEGIN_EMPTY_NAMESPACE

class FaceDetectorTest : public ::testing::Test
{
protected:

    // Setup
    FaceDetectorTest()
    {
        m_logger = drishti::core::Logger::create("test-drishti-face");
        m_logger->set_level(spdlog::level::off); // by default...

        // Load the ground truth data:
        image = loadImage(sFaceImageFilename);

        // Create FaceDetectorFactory (default file based):
        m_factory = std::make_shared<drishti::face::FaceDetectorFactory>();
        m_factory->sFaceDetector = sFaceDetector;
        m_factory->sFaceRegressor = sFaceRegressor;
        m_factory->sEyeRegressor = sEyeRegressor;
        m_factory->sFaceDetectorMean = sFaceDetectorMean;

        m_detector = std::make_shared<drishti::face::FaceDetectorAndTracker>(*m_factory);
        m_detector->setScaling(1.f);
        m_detector->setDoNMS(true);
        m_detector->setDoNMSGlobal(true);
    }

    // Cleanup
    virtual ~FaceDetectorTest()
    {
        drishti::core::Logger::drop("test-drishti-face");
    }

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    cv::Mat loadImage(const std::string& filename)
    {
        assert(!filename.empty());
        cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);

        cv::Mat Irgb;
        cv::cvtColor(image, Irgb, cv::COLOR_BGR2RGB);    

        cv::Mat It = Irgb.t(), Itf;
        It.convertTo(Itf, CV_32FC3, 1.0f / 255.f);
        Ip = MatP(Itf);

        cv::Mat green;
        cv::extractChannel(image, green, 1);
        Ib = drishti::face::FaceDetector::PaddedImage(green, { { 0, 0 }, green.size() });
        
        return image;
    }

    cv::Mat image;
    MatP Ip;
    drishti::face::FaceDetector::PaddedImage Ib;

    std::shared_ptr<spdlog::logger> m_logger;
    std::shared_ptr<drishti::face::FaceDetectorFactory> m_factory;
    std::shared_ptr<drishti::face::FaceDetectorAndTracker> m_detector;
};

TEST_F(FaceDetectorTest, FaceDetector)
{
    std::vector<drishti::face::FaceModel> faces;
    (*m_detector)(Ip, Ib, faces, cv::Matx33f::eye());
    ASSERT_EQ(faces.size(), 1);
}

#if defined(DRISHTI_BUILD_EOS)
TEST_F(FaceDetectorTest, FaceMeshMapper)
{
    std::vector<drishti::face::FaceModel> faces;
    (*m_detector)(Ip, Ib, faces, cv::Matx33f::eye());
    ASSERT_EQ(faces.size(), 1);

    // Fit landmarks here
}
#endif

END_EMPTY_NAMESPACE
