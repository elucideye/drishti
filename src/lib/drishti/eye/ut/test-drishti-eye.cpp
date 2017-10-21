/*! -*-c++-*-
  @file   test-drishti-eye.cpp
  @author David Hirvonen
  @brief  Google fixture with various subtests for the drishti public SDK.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>

#include "drishti/core/drishti_serialize.h"

// These must come before drishti_cv.hpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <gtest/gtest.h>

#include <algorithm>
#include <fstream>
#include <iomanip>

// https://code.google.com/p/googletest/wiki/Primer

extern const char* sEyeRegressor;
extern const char* sEyeImageFilename;
extern const char* sEyeModelPrivateFilename;
extern const char* sOutputDirectory;

#define BEGIN_EMPTY_NAMESPACE \
    namespace                 \
    {
#define END_EMPTY_NAMESPACE }

#define EYE_ASPECT_RATIO (4.0 / 3.0)

BEGIN_EMPTY_NAMESPACE

static cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio);
static float detectionScore(const drishti::eye::EyeModel& eyeA, const drishti::eye::EyeModel& eyeB, const cv::Size& size, float scale);

class EyeModelEstimatorTest : public ::testing::Test
{
public:
    static std::shared_ptr<drishti::eye::EyeModelEstimator> create(const std::string& filename)
    {
        auto segmenter = std::make_shared<drishti::eye::EyeModelEstimator>(filename);
        return (segmenter && segmenter->good()) ? segmenter : nullptr;
    }

protected:
    struct Entry
    {
        cv::Mat image;
        bool isRight;
    };

    // Setup
    EyeModelEstimatorTest()
    {
        // Create the segmenter (constructor tests performed prior to this)
        m_eyeSegmenter = create(sEyeRegressor);

        // Load the ground truth data:
        loadTruth();

        // Load sample for each image width:
        loadImages();
    }

    // Cleanup
    virtual ~EyeModelEstimatorTest()
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

        m_image = image;

        cv::Mat padded;
        cv::Rect roi({ 0, 0 }, image.size());
        padToAspectRatio(image, padded, EYE_ASPECT_RATIO);
        assert(!padded.empty());

        std::vector<int> widths = { 0 }; // first one is empty
#if defined(GAUZE_ANDROID_USE_EMULATOR)
        for (int width = 16; width <= 256; width *= 2)
        {
            widths.push_back(width);
        }
#else
        for (int width = 16; width <= 256; width++)
        {
            widths.push_back(width);
        }
#endif

        for (const auto& width : widths)
        {
            cv::Mat resized;
            if (width > 0)
            {
                const int height = static_cast<int>(static_cast<float>(width) / EYE_ASPECT_RATIO + 0.5f);
                cv::resize(padded, resized, { width, height }, width);
            }

            m_images[width] = { resized, true };
        }
    }

    void loadTruth()
    {
        assert(sEyeModelPrivateFilename);
        std::ifstream is(sEyeModelPrivateFilename);
        if (is.good())
        {
            auto eye = std::make_shared<drishti::eye::EyeModel>();
            cereal::JSONInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia(GENERIC_NVP("eye", *eye));
            m_eye = eye;
            m_eye->refine();
        }
        else
        {
            std::cerr << "Unable to find ground truth file: " << sEyeModelPrivateFilename << std::endl;
            std::cerr << "Make sure test-drishti-drishti is run first." << std::endl;
        }
    }

    void createImage(Entry& entry, int rows, int cols, const cv::Vec3b& color)
    {
        entry.image.create(rows, cols, CV_8UC3);
        entry.isRight = true;
    }

    std::map<int, Entry>::iterator getFirstGreat(int width = 32)
    {
        auto isok = [=](const std::pair<int, Entry>& e) { return e.second.image.cols >= width; };
        return std::find_if(m_images.begin(), m_images.end(), isok);
    }

    // Objects declared here can be used by all tests in the test case for EyeModelEstimator.
    std::shared_ptr<drishti::eye::EyeModelEstimator> m_eyeSegmenter;

    // Test images:
    std::map<int, Entry> m_images;

    int m_targetWidth = 128;

    // Ground truth image:
    cv::Mat m_image;

    // Ground truth:
    std::shared_ptr<drishti::eye::EyeModel> m_eye;

    // Score tolerance:
    float m_scoreThreshold = 0.70;
};

static void checkValid(const drishti::eye::EyeModel& eye, const cv::Size& size)
{
    EXPECT_GT(eye.roi->width, 0);
    EXPECT_GT(eye.roi->height, 0);
    EXPECT_GE(eye.eyelids.size(), 16);
    EXPECT_GT(eye.irisEllipse.size.width, 0);
    EXPECT_GT(eye.pupilEllipse.size.width, 0);
}

static bool isEqual(const drishti::eye::EyeModel& eyeA, const drishti::eye::EyeModel& eyeB)
{
    return eyeA == eyeB;
}

/*
 * Basic class construction
 */

TEST(EyeModelEstimator, StringConstructor)
{
    if (isArchiveSupported(sEyeModelPrivateFilename))
    {
        ASSERT_NE(sEyeModelPrivateFilename, (const char*)NULL);
        auto segmenter = EyeModelEstimatorTest::create(sEyeModelPrivateFilename);
        ASSERT_EQ(segmenter->good(), true);
    }
}

TEST(EyeModelEstimator, StreamConstructor)
{
    if (isArchiveSupported(sEyeModelPrivateFilename))
    {
        ASSERT_NE(sEyeModelPrivateFilename, (const char*)NULL);
        std::ifstream is(sEyeModelPrivateFilename);
        ASSERT_TRUE((bool)is);
        auto segmenter = EyeModelEstimatorTest::create(sEyeModelPrivateFilename);
        EXPECT_EQ(segmenter->good(), true);
    }
}

TEST_F(EyeModelEstimatorTest, CerealSerialization)
{
    std::string filename = std::string(sOutputDirectory) + "/eye.cpb";
    save_cpb(filename, *m_eyeSegmenter);

    drishti::eye::EyeModelEstimator segmenter2;
    load_cpb(filename, segmenter2);
}

/*
 * Fixture tests
 */

TEST_F(EyeModelEstimatorTest, EyeSerialization)
{
    drishti::eye::EyeModel eye;

    assert(m_images[m_targetWidth].isRight);
    /* int code = */ (*m_eyeSegmenter)(m_images[m_targetWidth].image, eye);

    std::string filename(sOutputDirectory);
    filename += "/right_eye_test.json";

    std::ofstream os(filename);
    cereal::JSONOutputArchive oa(os);
    typedef decltype(oa) Archive;
    oa << GENERIC_NVP("eye", eye);
}

// * hamming distance for sclera and iris masks components
TEST_F(EyeModelEstimatorTest, ImageValid)
{
    if (!m_eye || !m_eyeSegmenter)
    {
        return;
    }

    for (auto iter = getFirstGreat(); iter != m_images.end(); iter++)
    {
        // Make sure image has the expected size:
        EXPECT_EQ(iter->second.image.cols, iter->first);

        assert(iter->second.isRight);
        drishti::eye::EyeModel eye;
        int code = (*m_eyeSegmenter)(iter->second.image, eye);
        EXPECT_EQ(code, 0);

        eye.refine();
        checkValid(eye, iter->second.image.size());

        // Ground truth comparison for reasonable resolutions
        if (iter->first >= 128)
        {
            const float scaleGroundTruthToCurrent = static_cast<float>(iter->second.image.cols) / static_cast<float>(m_targetWidth);
            const float score = detectionScore(*m_eye, eye, iter->second.image.size(), scaleGroundTruthToCurrent);
            ASSERT_GT(score, m_scoreThreshold);
        }
    }
}

TEST_F(EyeModelEstimatorTest, IsRepeatable)
{
    if (!m_eye || !m_eyeSegmenter)
    {
        return;
    }

    for (auto iter = getFirstGreat(); iter != m_images.end(); iter++)
    {
        // Make sure image has the expected size:
        EXPECT_EQ(iter->second.image.cols, iter->first);

        assert(iter->second.isRight);
        drishti::eye::EyeModel eyeA, eyeB;

        int codeA = (*m_eyeSegmenter)(iter->second.image, eyeA);
        EXPECT_EQ(codeA, 0);
        eyeA.refine();
        checkValid(eyeA, iter->second.image.size());

        int codeB = (*m_eyeSegmenter)(iter->second.image, eyeB);
        EXPECT_EQ(codeB, 0);
        eyeB.refine();
        checkValid(eyeB, iter->second.image.size());

        EXPECT_EQ(isEqual(eyeA, eyeB), true);
    }
}

// Currently there is no internal quality check, but this is included for regression:
TEST_F(EyeModelEstimatorTest, ImageIsBlack)
{
    Entry entry;
    const int width = 128;
    const int height = static_cast<int>(static_cast<float>(width) / EYE_ASPECT_RATIO + 0.5f);
    createImage(entry, height, width, { 0, 0, 0 });

    assert(entry.isRight);
    drishti::eye::EyeModel eye;
    int code = (*m_eyeSegmenter)(entry.image, eye);
    EXPECT_EQ(code, 0);
    checkValid(eye, entry.image.size());
}

TEST_F(EyeModelEstimatorTest, ImageIsWhite)
{
    Entry entry;
    const int width = 128;
    const int height = static_cast<int>(static_cast<float>(width) / EYE_ASPECT_RATIO + 0.5f);
    createImage(entry, height, width, { 0, 0, 0 });

    assert(entry.isRight);
    drishti::eye::EyeModel eye;
    int code = (*m_eyeSegmenter)(entry.image, eye);
    EXPECT_EQ(code, 0);
    checkValid(eye, entry.image.size());
}

// #######

static cv::Mat scleraMask(const drishti::eye::EyeModel& eye, const cv::Size& size)
{
    return eye.mask(size);
}

static cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio)
{
    CV_Assert(image.channels() == 3);

    int top = 0, left = 0, bottom = 0, right = 0;
    if (double(image.cols) / image.rows > aspectRatio)
    {
        int padding = static_cast<int>(static_cast<double>(image.cols) / aspectRatio + 0.5) - image.rows;
        top = padding / 2;
        bottom = padding - top;
    }
    else
    {
        int padding = static_cast<int>(static_cast<double>(image.rows) * aspectRatio + 0.5) - image.cols;
        left = padding / 2;
        right = padding - left;
    }

    cv::Scalar mu = cv::mean(image);
    cv::copyMakeBorder(image, padded, top, bottom, left, right, cv::BORDER_CONSTANT, mu);

    return cv::Point(left, top);
}

// eyeA: ground truth eye
// eyeB: test eye
static float
detectionScore(const drishti::eye::EyeModel& eyeA, const drishti::eye::EyeModel& eyeB, const cv::Size& size, float scale)
{
    drishti::eye::EyeModel eyeGT = eyeA * scale;
    cv::Mat maskGT = scleraMask(eyeGT, size);
    cv::Mat maskB = scleraMask(eyeB, size);

    int numerator = 0, denominator = 0;
    try
    {
        numerator = cv::countNonZero(maskGT & maskB);
    }
    catch (...)
    {
        // opencv throws when empty
    }
    try
    {
        denominator = cv::countNonZero(maskGT | maskB);
    }
    catch (...)
    {
        // opencv throws when empty
    }
    float score = denominator ? float(numerator) / (denominator) : 0;

    return score;
}

END_EMPTY_NAMESPACE
