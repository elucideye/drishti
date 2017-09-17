/*! -*-c++-*-
  @file   videoio.cpp
  @author David Hirvonen
  @brief  Video to stills IO test.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "VideoSourceCV.h"

#include "drishti/core/Logger.h"

#include "cxxopts.hpp"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include <iomanip>
#include <iostream>

int gauze_main(int argc, char** argv)
{
    auto logger = drishti::core::Logger::create("test-videoio");

    std::string sInput, sOutput;
    cxxopts::Options options("test-videoio", "Dump video frames to stills.");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);
    if (sInput.empty() || sOutput.empty())
    {
        logger->info("Must specify a valid input and output file");
    }

    auto video = drishti::videoio::VideoSourceCV::create(sInput);
    if (video == nullptr)
    {
        logger->error("Failed to read video {}", sInput);
        return -1;
    }

    drishti::videoio::VideoSourceCV::Frame frame;
    for (int i = 0; i < video->count(); i++)
    {
        frame = (*video)(i);
        if (frame.image.empty())
        {
            break;
        }

        std::stringstream ss;
        ss << sOutput << "/frame_" << std::setw(4) << std::setfill('0') << i++ << ".png";
        cv::imwrite(ss.str(), frame.image);
    }
    while (!frame.image.empty())
        ;

    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    catch (...)
    {
        std::cerr << "Unknown exception";
    }
}
