/*!
  @file   test-drishti-face.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Note that drishti::face:FaceFinder has a dependency on an active OpenGL rendering context,
  which is used (at the least) for creating upright rescaled video frames for ACF channel 
  computation, or (at the most) full ACF channel computation on the GPU using shaders.

*/

#include <gtest/gtest.h>

#include "drishti/face/FaceDetectorAndTracker.h"

const char *sFaceDetector;
const char *sFaceDetectorMean;
const char *sFaceRegressor;
const char *sEyeRegressor;

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);

    assert(argc == 5);

    sFaceDetector = argv[1];
    sFaceDetectorMean = argv[2];
    sFaceRegressor = argv[3];
    sEyeRegressor = argv[4];
    
    return RUN_ALL_TESTS();
}

TEST(FaceDetectorAndTracker, Instantiation)
{
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();
    factory->sFaceDetector = sFaceDetector;
    factory->sFaceRegressors = { sFaceRegressor };
    factory->sEyeRegressor = sEyeRegressor; 
    factory->sFaceDetectorMean = sFaceDetectorMean;

    drishti::face::FaceDetectorAndTracker detector(*factory);
    
    ASSERT_EQ(true, true);
}
