/*!
  @file   test-drishti-ml.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

extern const char* modelFilename;
extern const char* imageFilename;
extern const char* truthFilename;
extern const char* outputDirectory;
extern bool isTextArchive;

#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/ml/XGBooster.h"

#if DRISHTI_SERIALIZE_WITH_BOOST
#  include "drishti/core/drishti_serialize.h"
#  include "drishti/core/boost_serialize_common.h"
#endif

#if DRISHTI_SERIALIZE_WITH_CEREAL
#  include "drishti/core/drishti_stdlib_string.h"
#  include "drishti/core/drishti_cereal_pba.h"
#  include "drishti/core/drishti_cv_cereal.h"
#endif

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc >= 4);
    modelFilename = argv[1];
    imageFilename = argv[2];
    truthFilename = argv[3];
    outputDirectory = argv[4];
    isTextArchive = (argc > 5) ? (std::atoi(argv[5]) > 0) : false;    
    return RUN_ALL_TESTS();
}

TEST(XGBooster, XGBoosterInit)
{
    drishti::ml::XGBooster booster;
    ASSERT_EQ(true, true);
}
