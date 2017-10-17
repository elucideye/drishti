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
        padToAspectRatio(image, padded, 4.0 / 3.0);
        assert(!padded.empty());

        // Next create each possible size from 0 to 1000:
        for (int width = 0; width < 256; width++)
        {
            cv::Mat resized;
            if (width > 0)
            {
                int height(float(width) / EYE_ASPECT_RATIO + 0.5f);
                cv::resize(padded, resized, { width, height }, width);
            }

            {
                // Create right image
                Entry entry{ resized, true };
                m_images.emplace_back(entry);
            }
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

    // Objects declared here can be used by all tests in the test case for EyeModelEstimator.
    std::shared_ptr<drishti::eye::EyeModelEstimator> m_eyeSegmenter;

    // Test images:
    std::vector<Entry> m_images;

    int m_targetWidth = 127;

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
    if (m_eyeSegmenter)
    {
        std::string filename = std::string(sOutputDirectory) + "/eye.cpb";
        save_cpb(filename, *m_eyeSegmenter);

        drishti::eye::EyeModelEstimator segmenter2;
        load_cpb(filename, segmenter2);
    }
}

/*
 * Fixture tests
 */

TEST_F(EyeModelEstimatorTest, EyeSerialization)
{
    if (m_eyeSegmenter)
    {
        drishti::eye::EyeModel eye;

        assert(m_images[m_targetWidth].isRight);
        /* int code = */ (*m_eyeSegmenter)(m_images[m_targetWidth].image, eye);

        std::string filename(sOutputDirectory);
        filename += "/right_eye_2.json";

        std::ofstream os(filename);
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa << GENERIC_NVP("eye", eye);
    }
}

// * hamming distance for sclera and iris masks components
TEST_F(EyeModelEstimatorTest, ImageValid)
{
    if (!m_eye || !m_eyeSegmenter)
    {
        return;
    }

    for (int i = 32; i < m_images.size(); i++)
    {
        // Make sure image has the expected size:
        EXPECT_EQ(m_images[i].image.cols, i);

        assert(m_images[i].isRight);
        drishti::eye::EyeModel eye;
        int code = (*m_eyeSegmenter)(m_images[i].image, eye);
        EXPECT_EQ(code, 0);

        eye.refine();
        checkValid(eye, m_images[i].image.size());

        // Ground truth comparison for reasonable resolutions
        if (i > 100)
        {
            const float scaleGroundTruthToCurrent = float(m_images[i].image.cols) / float(m_targetWidth);
            const float score = detectionScore(*m_eye, eye, m_images[i].image.size(), scaleGroundTruthToCurrent);
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

    for (int i = 64; i < m_images.size(); i++)
    {
        // Make sure image has the expected size:
        EXPECT_EQ(m_images[i].image.cols, i);

        assert(m_images[i].isRight);
        drishti::eye::EyeModel eyeA, eyeB;

        int codeA = (*m_eyeSegmenter)(m_images[i].image, eyeA);
        EXPECT_EQ(codeA, 0);
        eyeA.refine();
        checkValid(eyeA, m_images[i].image.size());

        int codeB = (*m_eyeSegmenter)(m_images[i].image, eyeB);
        EXPECT_EQ(codeB, 0);
        eyeB.refine();
        checkValid(eyeB, m_images[i].image.size());

        EXPECT_EQ(isEqual(eyeA, eyeB), true);
    }
}

// Currently there is no internal quality check, but this is included for regression:
TEST_F(EyeModelEstimatorTest, ImageIsBlack)
{
    if (m_eyeSegmenter)
    {
        Entry entry;
        int width = 256;
        int height = int(float(width) / EYE_ASPECT_RATIO + 0.5f);
        createImage(entry, height, width, { 0, 0, 0 });

        assert(entry.isRight);
        drishti::eye::EyeModel eye;
        int code = (*m_eyeSegmenter)(entry.image, eye);
        EXPECT_EQ(code, 0);
        checkValid(eye, entry.image.size());
    }
}

TEST_F(EyeModelEstimatorTest, ImageIsWhite)
{
    if (m_eyeSegmenter)
    {
        Entry entry;
        int width = 256;
        int height = int(float(width) / EYE_ASPECT_RATIO + 0.5f);
        createImage(entry, height, width, { 0, 0, 0 });

        assert(entry.isRight);
        drishti::eye::EyeModel eye;
        int code = (*m_eyeSegmenter)(entry.image, eye);
        EXPECT_EQ(code, 0);
        checkValid(eye, entry.image.size());
    }
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
    }
    try
    {
        denominator = cv::countNonZero(maskGT | maskB);
    }
    catch (...)
    {
    }
    float score = denominator ? float(numerator) / (denominator) : 0;

#define DEBUG_PASCAL 0
#if DEBUG_PASCAL
    {
        std::cout << "SCORE: " << score << std::endl;
        cv::imshow("maskA", maskGT); // opt
        cv::imshow("maskB", maskB);  // opt
        int i = 0;
        cv::Mat tmp[2] = { maskGT, maskB };
        do
        {
            cv::imshow("a", tmp[i++ % 2]);
        } while (cv::waitKey(0) != int('q'));
        cv::waitKey(0);
    }
#endif

    return score;
}

END_EMPTY_NAMESPACE
