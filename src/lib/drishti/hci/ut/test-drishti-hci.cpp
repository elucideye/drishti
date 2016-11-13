/*!
  @file   test-hci.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Note that drishti::hci:FaceFinder has a dependency on an active OpenGL rendering context,
  which is used (at the least) for creating upright rescaled video frames for ACF channel 
  computation, or (at the most) full ACF channel computation on the GPU using shaders.

*/

#include <gtest/gtest.h>

#include "drishti/hci/FaceFinder.h"

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

TEST(FaceFinder, Instantiation)
{
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();
    factory->sFaceDetector = sFaceDetector;
    factory->sFaceRegressors = { sFaceRegressor };
    factory->sEyeRegressor = sEyeRegressor; 
    factory->sFaceDetectorMean = sFaceDetectorMean;

    drishti::hci::FaceFinder::Config config;
    drishti::hci::FaceFinder faceFinder(factory, config, nullptr);
    
    ASSERT_EQ(true, true);
}
