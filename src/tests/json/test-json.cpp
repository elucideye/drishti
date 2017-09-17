/*! -*-c++-*-
  @file   test-json.cpp
  @author David Hirvonen
  @brief  Google test for nlohmann-json package.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_stdlib_string.h"
#include "nlohmann/json.hpp" // nlohman-json

#include <gtest/gtest.h>

#include <stdlib.h>
#include <fstream>

const char* filename;

int gauze_main(int argc, char** argv)
{
    assert(argc == 2);
    filename = argv[1];

    std::cout << "FILENAME: " << filename << std::endl;

    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

TEST(NLohmannJson, NLohmannJSonParse)
{
    static const char* sName = "com.apple.avfoundation.avcapturedevice.built-in_video:1";
    const std::string name(sName);

    std::ifstream ifs(filename, std::ifstream::in);
    nlohmann::json json;
    ASSERT_EQ(ifs.is_open(), true);
    if (!ifs.is_open())
    {
        std::cerr << "Failed to open test file: " << filename << std::endl;
        return;
    }

    ifs >> json;
    const auto& device = json[name];

    ASSERT_EQ(device.empty(), false);
    if (device.empty())
    {
        std::cerr << "Failure to parse settings for device:" << name;
        return;
    }

    float minDepth = device["detectionRange"]["minDepth"];
    ASSERT_FLOAT_EQ(minDepth, 0.1f);
    float maxDepth = device["detectionRange"]["maxDepth"];
    ASSERT_FLOAT_EQ(maxDepth, 0.5f);

    const auto& sensor = device["sensor"];
    const auto& intrinsic = sensor["intrinsic"];

    int width = intrinsic["size"]["width"].get<int>();
    ASSERT_EQ(width, 1280);
    int height = intrinsic["size"]["height"].get<int>();
    ASSERT_EQ(height, 720);

    float x = intrinsic["principal"]["x"].get<float>();
    ASSERT_FLOAT_EQ(x, 640.f);
    float y = intrinsic["principal"]["y"].get<float>();
    ASSERT_FLOAT_EQ(y, 360.f);
}
