/*!
  @file   test-RTEShapeEstimator.cpp
  @author David Hirvonen
  @brief  Google fixture with various subtests for the RegressionTreeEnsembleShapeEstimator

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

// These must come before drishti_cv.hpp
#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#if DRISHTI_SERIALIZE_WITH_BOOST
#  include "drishti/core/drishti_serialize.h"
#  include "drishti/core/boost_serialize_common.h"
#endif

#if DRISHTI_SERIALIZE_WITH_CEREAL
#  include "drishti/core/drishti_stdlib_string.h"
#  include "drishti/core/drishti_cereal_pba.h"
#  include "drishti/core/drishti_cv_cereal.h"
//include <cereal/archives/json.hpp>
//include <cereal/archives/xml.hpp>
#endif

#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"

#include <fstream>

// https://code.google.com/p/googletest/wiki/Primer

const char* modelFilename;
const char* imageFilename;
const char* truthFilename;
const char* outputDirectory;
bool isTextArchive;

#define BEGIN_EMPTY_NAMESPACE namespace {
#define END_EMPTY_NAMESPACE }

BEGIN_EMPTY_NAMESPACE

class RTEShapeEstimatorTest : public ::testing::Test
{
public:
    
    static std::shared_ptr<drishti::ml::RegressionTreeEnsembleShapeEstimator> create(const std::string &filename, bool textArchive)
    {
        return std::make_shared<drishti::ml::RegressionTreeEnsembleShapeEstimator>(filename);
    }
    
protected:

    // Setup
    RTEShapeEstimatorTest()
    {
        // Create the segmenter (constructor tests performed prior to this)
        m_shapePredictor = create(modelFilename, isTextArchive);

        // Load the ground truth data:
        loadTruth();

        // Load sample for each image width:
        loadImages();
    }

    // Cleanup
    virtual ~RTEShapeEstimatorTest() {}

    // Called after constructor for each test
    virtual void SetUp() {}

    // Called after destructor for each test
    virtual void TearDown() {}

    // Utility methods:
    void loadImages()
    {
        assert(imageFilename);

        // First get our 4x3 aspect ratio image
        cv::Mat image = cv::imread(imageFilename, cv::IMREAD_GRAYSCALE);
        assert(!image.empty());
        
        m_image = image;
    }

    void loadTruth()
    {
        assert(truthFilename);
        std::ifstream is(truthFilename);
        if(is)
        {
            // TODO: Load ground truth model here
        }
        else
        {
            std::cerr << "Unable to find ground truth file: " << truthFilename << std::endl;
        }
    }
    
    // Objects declared here can be used by all tests in the test case for RTEShapeEstimator.
    std::shared_ptr<drishti::ml::RegressionTreeEnsembleShapeEstimator> m_shapePredictor;

    int m_targetWidth = 127;
    
    // Ground truth image:
    cv::Mat m_image;

    // Score tolerance:
    float m_scoreThreshold = 0.60;
};

/*
 * Basic class construction
 */

#if DRISHTI_SERIALIZE_WITH_BOOST
TEST(RTEShapeEstimator, StringConstructor)
{
    // Make sure modelFilename is not null:
    ASSERT_NE(modelFilename, (const char *)NULL);

    auto predictor = RTEShapeEstimatorTest::create(modelFilename, isTextArchive);
    
    // Make sure we reached this point (no exceptions):
    ASSERT_NE(predictor, nullptr);
}
#endif // DRISHTI_SERIALIZE_WITH_BOOST

#if DRISHTI_SERIALIZE_WITH_BOOST
TEST(RTEShapeEstimator, StreamConstructor)
{
    // Make sure modelFilename is not null:
    ASSERT_NE(modelFilename, (const char *)NULL);

    std::ifstream is(modelFilename, std::ios::binary);

    // Make sure file could be opened:
    ASSERT_TRUE((bool)is);

    auto predictor = std::make_shared<drishti::ml::RegressionTreeEnsembleShapeEstimator>(is);

    // Make sure we reached this point (no exceptions):
    ASSERT_NE(predictor, nullptr);
}
#endif // DRISHTI_SERIALIZE_WITH_BOOST

#if DRISHTI_SERIALIZE_WITH_CEREAL
TEST_F(RTEShapeEstimatorTest, CerealSerialization)
{
    std::string filename = std::string(outputDirectory) + "/shape.cpb";
    {
        std::ofstream ofs(filename, std::ios::binary);
        if(ofs)
        {
            cereal::PortableBinaryOutputArchive oa(ofs);
            oa << *m_shapePredictor;
        }
    }
    
    drishti::ml::RegressionTreeEnsembleShapeEstimator estimator2;
    {
        std::ifstream ifs(filename, std::ios::binary);
        if(ifs)
        {
            cereal::PortableBinaryInputArchive3 ia(ifs);
            ia >> estimator2;
        }
    }   
}
#endif // DRISHTI_SERIALIZE_WITH_CEREAL

/*
 * Fixture tests
 */

TEST_F(RTEShapeEstimatorTest, ShapeSerialization)
{
    std::vector<bool> mask;
    std::vector<cv::Point2f> points;
    /* int code = */ (*m_shapePredictor)(m_image, points, mask);
}

END_EMPTY_NAMESPACE

