/*!
  @file   test-FaceTracker.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/drishti/FaceTracker.hpp"
#include "drishti/drishti/Manager.hpp"

#include <memory>
#include <fstream>

const char *sFaceDetector;
const char *sFaceDetectorMean;
const char *sFaceRegressor;
const char *sEyeRegressor;

int main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc == 4);
    sFaceDetector = argv[1];
    sFaceDetectorMean = argv[2];
    sFaceRegressor = argv[3];
    sEyeRegressor = argv[4];
    return RUN_ALL_TESTS();
}

TEST(FaceTracker, Instantiation)
{
    drishti::sensor::SensorModel sensor;
    drishti::sdk::Manager manager(sensor);
    drishti::sdk::FaceTracker::Resources factory;

    std::ifstream iFaceDetector(sFaceDetector, std::ios_base::binary);
    std::ifstream iFaceRegressor(sFaceRegressor, std::ios_base::binary);
    std::ifstream iEyeRegressor(sEyeRegressor, std::ios_base::binary);
    std::ifstream iFaceDetectorMean(sFaceDetectorMean, std::ios_base::binary);
        
    factory.sFaceDetector = &iFaceDetector;
    factory.sFaceRegressors = { &iFaceRegressor };
    factory.sEyeRegressor = &iEyeRegressor;
    factory.sFaceModel = &iFaceDetectorMean;
    
    auto tracker = std::make_shared<drishti::sdk::FaceTracker>(&manager, factory);
    
    ASSERT_EQ(true, true);
}
