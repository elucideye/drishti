/*! -*-c++-*-
  @file   drishti-example-eye.cpp
  @author David Hirvonen
  @brief  Drishti SDK test for eye model fitting.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/EyeSegmenter.hpp"
#include "drishti/drishti_cv.hpp"

#include <opencv2/highgui.hpp>

#include <string>

static bool toBool(const char* str);

int gauze_main(int argc, char** argv)
{
    for (int i = 0; i < argc; i++)
    {
        std::cerr << "argv[" << i << "]=" << argv[i] << std::endl;
    }

    if (argc != 5)
    {
        std::cerr << "Usage: drishti-example-eye <model> <input> <is-right> <output-dir>" << std::endl;
        exit(1);
    }

    const std::string model = argv[1];
    const std::string filename = argv[2];
    const bool isRight = toBool(argv[3]);
    const std::string output = argv[4];

    drishti::sdk::EyeSegmenter segmenter(model);
    if (!segmenter)
    {
        std::cerr << "Unable to load model specified in: " << model << std::endl;
        exit(1);
    }

    cv::Mat image = cv::imread(filename, cv::IMREAD_COLOR);
    if (image.empty())
    {
        std::cerr << "Unable to load image specified in: " << filename << std::endl;
        exit(1);
    }

    auto image_ = drishti::sdk::cvToDrishti<cv::Vec3b, drishti::sdk::Vec3b>(image);
    drishti::sdk::Eye eye;
    segmenter(image_, eye, isRight);

    cv::Mat1b mask(image.size(), 0);
    auto mask_ = drishti::sdk::cvToDrishti<uint8_t, uint8_t>(mask);
    drishti::sdk::createMask(mask_, eye, static_cast<int>(drishti::sdk::kScleraRegion));

    cv::imwrite(output + "/mask.png", mask);

    return 0;
}

static int string_to_int(const std::string& str)
{
#if ANDROID // add cmake system check
    return atoi(str.c_str());
#else
    return std::stoi(str);
#endif
}

static bool toBool(const char* str)
{
    bool value = true;

    std::string arg(str);
    if (string_to_int(arg) == 0 || arg == "false" || arg == "FALSE")
    {
        value = false;
    }
    else if (string_to_int(arg) == 1 || arg == "true" || arg == "TRUE")
    {
        value = true;
    }
    else
    {
        std::cerr << "Invalid boolean specified in: " << str << " assuming " << value << std::endl;
    }

    return value;
}
