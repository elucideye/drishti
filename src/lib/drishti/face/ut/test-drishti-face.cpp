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

#include <gtest/gtest.h>

#include "drishti/face/FaceDetectorAndTracker.h"

extern const char * sFaceDetector;
extern const char * sFaceDetectorMean;
extern const char * sFaceRegressor;
extern const char * sEyeRegressor;

TEST(FaceDetectorAndTracker, Instantiation)
{
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();
    factory->sFaceDetector = sFaceDetector;
    factory->sFaceRegressor = sFaceRegressor;
    factory->sEyeRegressor = sEyeRegressor;
    factory->sFaceDetectorMean = sFaceDetectorMean;

    drishti::face::FaceDetectorAndTracker detector(*factory);

    ASSERT_EQ(true, true);
}
