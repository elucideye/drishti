/*! -*-c++-*-
  @file   test-EyeSegmenter.cpp
  @author David Hirvonen
  @brief  Google fixture with various subtests for the drishti public SDK.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

// These must come before drishti_cv.hpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <fstream>

#include "drishti/EyeSegmenter.hpp"
#include "drishti/EyeIO.hpp"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/drishti_cv.hpp"
#include "drishti/EyeSegmenterImpl.hpp"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_serialize.h"

// clang-format off
#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
// clang-format on

// https://code.google.com/p/googletest/wiki/Primer
extern const char* sEyeRegressor;
extern const char* sEyeImageFilename;
extern const char* sEyeModelFilename;
extern const char* sOutputDirectory;

// clang-format off
#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }
// clang-format on

BEGIN_EMPTY_NAMESPACE

static cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio);
static float detectionScore(const drishti::sdk::Eye& eyeA, const drishti::sdk::Eye& eyeB);

static drishti::sdk::ArchiveKind getArchiveKind(const std::string& filename)
{
    return drishti::sdk::kCPB;
}

class EyeSegmenterTest : public ::testing::Test
{
public:
    static std::shared_ptr<drishti::sdk::EyeSegmenter> create(const std::string& filename)
    {
        if (isArchiveSupported(filename))
        {
            auto kind = getArchiveKind(filename);
            auto segmenter = std::make_shared<drishti::sdk::EyeSegmenter>(filename, kind);
            return (segmenter && segmenter->good()) ? segmenter : nullptr;
        }
        else
        {
            std::cerr << "DRISHTI_DRISHTI_TEST: archive not supported: " << filename << std::endl;
            return nullptr;
        }
    }

#if defined(DRISHTI_BUILD_C_INTERFACE)
    static std::shared_ptr<drishti::sdk::EyeSegmenter> createC(const std::string& filename)
    {
        if (isArchiveSupported(filename))
        {
            auto kind = getArchiveKind(filename);
            auto segmenter = drishti_eye_segmenter_create_from_file(filename, kind);

            if (segmenter)
            {
                if (segmenter->good())
                {
                    return std::shared_ptr<drishti::sdk::EyeSegmenter>(segmenter, drishti_eye_segmenter_destroy);
                }
                else
                {
                    delete segmenter;
                }
            }
            return nullptr;
        }
        else
        {
            std::cerr << "DRISHTI_DRISHTI_TEST: archive not supported: " << filename << std::endl;
            return nullptr;
        }
    }
#endif

protected:
    using Vec3b = drishti::sdk::Vec3b;
    using Image3b = drishti::sdk::Image3b;

    struct Entry
    {
        Image3b image;
        cv::Mat storage;
        bool isRight;
    };

    // Setup
    EyeSegmenterTest()
    {
        // Create the segmenter (constructor tests performed prior to this)
        m_eyeSegmenter = create(sEyeRegressor);

        // Load the ground truth data:
        loadTruth();

        // Load sample for each image width:
        loadImages();
    }

    // Cleanup
    virtual ~EyeSegmenterTest()
    {
    }

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    // Utility methods:
    void loadImages()
    {
        assert(sEyeImageFilename);

        // First get our 4x3 aspect ratio image
        cv::Mat image = cv::imread(sEyeImageFilename, cv::IMREAD_COLOR);
        assert(!image.empty());

        cv::Mat padded;
        cv::Rect roi({ 0, 0 }, image.size());
        padToAspectRatio(image, padded, 4.0 / 3.0);
        assert(!padded.empty());

        // Next create each possible size from 0 to 1000:
        for (int width = 0; width < 512; width++)
        {
            cv::Mat resized;
            if (width > 0)
            {
                int height(float(width) / m_eyeSegmenter->getRequiredAspectRatio() + 0.5f);
                cv::resize(padded, resized, { width, height }, width);
            }

            {
                // Create right image
                Entry entry{ drishti::sdk::cvToDrishti<cv::Vec3b, drishti::sdk::Vec3b>(resized), resized, true };
                m_images.emplace_back(entry);
            }
        }
    }

    void loadTruth()
    {
        drishti::sdk::Eye eye;
        drishti::sdk::EyeIStream adapter(eye, drishti::sdk::EyeStream::JSON);
        std::ifstream is(sEyeModelFilename);
        if (is)
        {
            is >> adapter;
            m_eye = std::make_shared<drishti::sdk::Eye>(eye);
        }
    }

    void createImage(Entry& entry, int rows, int cols, const Vec3b& color)
    {
        entry.storage.create(rows, cols, CV_8UC3);
        entry.image = drishti::sdk::cvToDrishti<cv::Vec3b, drishti::sdk::Vec3b>(entry.storage);
        entry.isRight = true;

        entry.storage = cv::Scalar(color[0], color[1], color[2]);
    }

    // Objects declared here can be used by all tests in the test case for EyeSegmenter.
    std::shared_ptr<drishti::sdk::EyeSegmenter> m_eyeSegmenter;

    // Test images:
    std::vector<Entry> m_images;

    // Ground truth:
    std::shared_ptr<drishti::sdk::Eye> m_eye;

    // Score tolerance:
    float m_scoreThreshold = 0.60;
};

/*
 * Basic class construction
 */

TEST(EyeSegmenter, StringConstructor)
{
    if (isArchiveSupported(sEyeRegressor))
    {
        ASSERT_NE(sEyeRegressor, (const char*)NULL);
        auto segmenter = EyeSegmenterTest::create(sEyeRegressor);
        ASSERT_EQ(segmenter && segmenter->good(), true);
    }
}

TEST(EyeSegmenter, StreamConstructor)
{
    if (isArchiveSupported(sEyeRegressor))
    {
        // Make sure sEyeRegressor is not null:
        ASSERT_NE(sEyeRegressor, (const char*)NULL);
        std::ifstream is(sEyeRegressor, std::ios_base::binary | std::ios::in);
        ASSERT_TRUE((bool)is);
        drishti::sdk::EyeSegmenter segmenter(is, getArchiveKind(sEyeRegressor));
        EXPECT_EQ(segmenter.good(), true);
    }
}

static void checkValid(const drishti::sdk::Eye& eye, const cv::Size& size)
{
    EXPECT_EQ(eye.getRoi().width, size.width);
    EXPECT_EQ(eye.getRoi().height, size.height);
    EXPECT_GE(eye.getEyelids().size(), 16);
    EXPECT_GT(eye.getIris().size.width, 0);
    EXPECT_GT(eye.getPupil().size.width, 0);
}

static void checkInvalid(const drishti::sdk::Eye& eye)
{
    EXPECT_EQ(eye.getEyelids().size(), 0);
    EXPECT_EQ(eye.getIris().size.width, 0);
    EXPECT_EQ(eye.getPupil().size.width, 0);
}

/*
 * Fixture tests
 */

TEST_F(EyeSegmenterTest, EyeSerialization)
{
    if (m_eyeSegmenter && sOutputDirectory)
    {
        int targetWidth = 127;
        drishti::sdk::Eye eye;
        /* int code = */ (*m_eyeSegmenter)(m_images[targetWidth].image, eye, m_images[targetWidth].isRight);

        {
            drishti::sdk::EyeOStream adapter(eye, drishti::sdk::EyeStream::JSON);
            std::string filename = sOutputDirectory;
            filename += "/right_eye.json";
            std::ofstream os(filename);
            if (os)
            {
                os << adapter;
            }
        }

        { // Convert this for use by private API
            auto privateEye = drishti::sdk::convert(eye);
            privateEye.eyelids = privateEye.eyelidsSpline;

            std::string filename = sOutputDirectory;
            filename += "/right_eye_private.json";

            std::ofstream os(filename);
            if (os)
            {
                cereal::JSONOutputArchive oa(os);
                typedef decltype(oa) Archive;
                oa << GENERIC_NVP("eye", privateEye);
            }
        }
    }
}

TEST_F(EyeSegmenterTest, ImageEmpty)
{
    if (m_eyeSegmenter)
    {
        // Requires at least 1 image:
        ASSERT_GE(m_images.size(), 1);

        // Make sure image is empty:
        ASSERT_EQ(m_images[0].image.getCols(), 0);

        drishti::sdk::Eye eye;
        int code = (*m_eyeSegmenter)(m_images[0].image, eye, m_images[0].isRight);
        EXPECT_EQ(code, 1);
        checkInvalid(eye);
    }
}

TEST_F(EyeSegmenterTest, ImageTooSmall)
{
    if (m_eyeSegmenter)
    {
        // Requires at least 1 image:
        EXPECT_GE(m_images.size(), m_eyeSegmenter->getMinWidth());

        for (int i = 1; i < m_eyeSegmenter->getMinWidth(); i++)
        {
            // Make sure image has the expected size:
            ASSERT_EQ(m_images[i].image.getCols(), i);

            drishti::sdk::Eye eye;
            int code = (*m_eyeSegmenter)(m_images[i].image, eye, m_images[i].isRight);
            EXPECT_EQ(code, 1);
            checkInvalid(eye);
        }
    }
}

// TODO: Add ground truth comparison
// * hamming distance for sclera and iris masks components
TEST_F(EyeSegmenterTest, ImageValid)
{
    if (m_eyeSegmenter)
    {
        // Requires at least m_eyeSegmenter->getMinWidth() + 1
        ASSERT_GT(m_images.size(), m_eyeSegmenter->getMinWidth());

        for (int i = m_eyeSegmenter->getMinWidth(); i < m_images.size(); i++)
        {
            // Make sure image has the expected size:
            EXPECT_EQ(m_images[i].image.getCols(), i);

            drishti::sdk::Eye eye;
            int code = (*m_eyeSegmenter)(m_images[i].image, eye, m_images[i].isRight);

            // Sanity check on each model:

            EXPECT_EQ(code, 0);
            checkValid(eye, m_images[i].storage.size());

            // Ground truth comparison for reasonable resolutions
            if (i > 128 && m_eye)
            {
                const float threshold = (i == m_eye->getRoi().width) ? m_scoreThreshold : 0.5;
                ASSERT_GT(detectionScore(eye, *m_eye), threshold);
            }
        }
    }
}

// Currently there is no internal quality check, but this is included for regression:
TEST_F(EyeSegmenterTest, ImageIsBlack)
{
    if (m_eyeSegmenter)
    {
        Entry entry;
        int width = 256;
        int height = int(float(width) / m_eyeSegmenter->getRequiredAspectRatio() + 0.5f);
        createImage(entry, height, width, { 0, 0, 0 });

        drishti::sdk::Eye eye;
        int code = (*m_eyeSegmenter)(entry.image, eye, entry.isRight);
        EXPECT_EQ(code, 0);
        checkValid(eye, entry.storage.size());
    }
}

TEST_F(EyeSegmenterTest, ImageIsWhite)
{
    if (m_eyeSegmenter)
    {
        Entry entry;
        int width = 256;
        int height = int(float(width) / m_eyeSegmenter->getRequiredAspectRatio() + 0.5f);
        createImage(entry, height, width, { 0, 0, 0 });

        drishti::sdk::Eye eye;
        int code = (*m_eyeSegmenter)(entry.image, eye, entry.isRight);
        EXPECT_EQ(code, 0);
        checkValid(eye, entry.storage.size());
    }
}

#if defined(DRISHTI_BUILD_C_INTERFACE)
TEST_F(EyeSegmenterTest, ExternCInterface)
{
    auto segmenter = createC(sEyeRegressor);

    ASSERT_NE(segmenter, nullptr);
    ASSERT_NE(m_eye, nullptr);    
    
    // Requires at least m_eyeSegmenter->getMinWidth() + 1
    ASSERT_GT(m_images.size(), m_eyeSegmenter->getMinWidth());

    for (int i = m_eyeSegmenter->getMinWidth(); i < m_images.size(); i++)
    {
        // Make sure image has the expected size:
        EXPECT_EQ(m_images[i].image.getCols(), i);

        drishti::sdk::Eye eye;
        int code = drishti_eye_segmenter_segment(segmenter.get(), m_images[i].image, eye, m_images[i].isRight);

        // Sanity check on each model:
        EXPECT_EQ(code, 0);
        checkValid(eye, m_images[i].storage.size());

        // Ground truth comparison for reasonable resolutions
        if (i > 128 && m_eye)
        {
            const float threshold = (i == m_eye->getRoi().width) ? m_scoreThreshold : 0.5;
            ASSERT_GT(detectionScore(eye, *m_eye), threshold);
        }
    }
}
#endif // DRISHTI_BUILD_C_INTERFACE

// #######

static cv::Mat scleraMask(const drishti::sdk::Eye& eye)
{
    using Vec2f = drishti::sdk::Vec2f;

    // TODO: deal with non zero offset
    assert(eye.getRoi().x == 0);
    assert(eye.getRoi().y == 0);

    cv::Mat mask = cv::Mat1b::zeros(drishtiToCv(eye.getRoi().size()));

    // Fill eyelid contour:
    std::vector<cv::Point2f> eyelids = drishti::sdk::drishtiToCv(eye.getEyelids());
    std::vector<std::vector<cv::Point>> contours(1);
    std::copy(eyelids.begin(), eyelids.end(), std::back_inserter(contours[0]));
    cv::fillPoly(mask, contours, 255);

    // Remove iris ellipse:
    cv::ellipse(mask, drishti::sdk::drishtiToCv(eye.getIris()), 0, -1, 8);

    return mask;
}

static cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio)
{
    CV_Assert(image.channels() == 3);

    int top = 0, left = 0, bottom = 0, right = 0;
    if (double(image.cols) / image.rows > aspectRatio)
    {
        int padding = int(double(image.cols) / aspectRatio + 0.5) - image.rows;
        top = padding / 2;
        bottom = padding - top;
    }
    else
    {
        int padding = int(double(image.rows) * aspectRatio + 0.5) - image.cols;
        left = padding / 2;
        right = padding - left;
    }

    cv::Scalar mu = cv::mean(image);
    cv::copyMakeBorder(image, padded, top, bottom, left, right, cv::BORDER_CONSTANT, mu);

    return cv::Point(left, top);
}

static float detectionScore(const drishti::sdk::Eye& eyeA, const drishti::sdk::Eye& eyeB)
{
    cv::Mat maskA = scleraMask(eyeA);
    cv::Mat maskB = scleraMask(eyeB);

    // Resize maskA to maskB coordinate system
    if (maskA.size() != maskB.size())
    {
        cv::resize(maskA, maskA, maskB.size(), 0, 0, cv::INTER_LANCZOS4);
        maskA = (maskA > 200);
    }

    int numerator = 0, denominator = 0;
    try
    {
        numerator = cv::countNonZero(maskA & maskB);
    }
    catch (...)
    {
        // opencv throws if count is 0
    }
    try
    {
        denominator = cv::countNonZero(maskA | maskB);
    }
    catch (...)
    {
        // opencv throws if count is 0        
    }
    float score = denominator ? float(numerator) / (denominator) : 0;

    return score;
}

END_EMPTY_NAMESPACE
