/*! -*-c++-*-
  @file   test-drishti.cpp
  @author David Hirvonen
  @brief  Google test for drishtisdk lib

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Note that drishti::hci:FaceFinder has a dependency on an active OpenGL rendering context,
  which is used (at the least) for creating upright rescaled video frames for ACF channel 
  computation, or (at the most) full ACF channel computation on the GPU using shaders.

*/

#include <gtest/gtest.h>

#include <cxxopts.hpp>

#include <fstream>

const char* sFaceDetector;
const char* sFaceDetectorMean;
const char* sFaceRegressor;
const char* sEyeRegressor;
const char* sEyeImageFilename;
const char* sEyeModelFilename;
const char* sEyeModelPrivateFilename;
const char* sFaceImageFilename;
const char* sFaceModelFilename;
const char* sOutputDirectory;

static bool hasFile(const std::string& filename)
{
    std::ifstream ifs(filename);
    return ifs.good();
}

int gauze_main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc == 11);

    sFaceDetector = argv[1];
    sFaceDetectorMean = argv[2];
    sFaceRegressor = argv[3];
    sEyeRegressor = argv[4];
    sEyeImageFilename = argv[5];
    sEyeModelFilename = argv[6];
    sEyeModelPrivateFilename = argv[7];    
    sFaceImageFilename = argv[8];
    sFaceModelFilename = argv[9];
    sOutputDirectory = argv[10];

    assert(hasFile(sFaceDetector));
    assert(hasFile(sFaceDetectorMean));
    assert(hasFile(sFaceRegressor));
    assert(hasFile(sEyeRegressor));
    assert(hasFile(sEyeImageFilename));
    assert(hasFile(sEyeModelFilename));
    assert(hasFile(sEyeModelPrivateFilename));
    assert(hasFile(sFaceImageFilename));
    assert(hasFile(sFaceModelFilename));
    
    return RUN_ALL_TESTS();
}
