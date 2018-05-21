/*!
  @file   drishti-eye-test.cpp
  @author David Hirvonen
  @brief  Test the drishti eye tracking SDK.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <drishti/EyeSegmenter.hpp>
#include <drishti/EyeIO.hpp>
#include <drishti/drishti_cv.hpp>

#include <opencv2/core.hpp>
#include <opencv2/highgui.hpp>

#include <cxxopts.hpp>

#include <spdlog/spdlog.h>

#include <fstream>

static std::shared_ptr<spdlog::logger> createLogger(const char* name);

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    bool isRight = false;
    bool isLeft = false;
    std::string sInput, sOutput, sModel;

    cxxopts::Options options("drishti-eye-test", "Command line interface for eye model fitting");

    // clang-format off
    options.add_options()
        // input/output:
        ("i,input", "Input image", cxxopts::value<std::string>(sInput))
        ("o,output", "Output image", cxxopts::value<std::string>(sOutput))
        ("m,model", "Eye model (pose regression)", cxxopts::value<std::string>(sModel))
        ("r,right", "Right eye", cxxopts::value<bool>(isRight))
        ("l,left", "Left eye", cxxopts::value<bool>(isLeft));
    // clang-format on    

    options.parse(argc, argv);
    if ((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return 0;
    }

    auto logger = createLogger("drishti-eye-test");
    
    if (sInput.empty())
    {
        logger->error("Must specify input {}", sInput);
        return 1;
    }

    if (sOutput.empty())
    {
        logger->error("Must specify output {}", sOutput);
        return 1;
    }

    if (sModel.empty())
    {
        logger->error("Must specify model {}", sModel);
        return 1;
    }

    // It is important that users are aware that the side matters.    
    // Be explicit: enforce CLI usage of --left or --right (even though redundant)
    if (!(isRight == !isLeft))
    {
        logger->error("Must specify left or right eye");
        return 1;
    }

    cv::Mat image = cv::imread(sInput, cv::IMREAD_COLOR);
    if (image.empty())
    {
        logger->error("Unable to read image {}", sInput);
        return 1;
    }

    drishti::sdk::EyeSegmenter segmenter(sModel);
    if(!segmenter)
    {
        logger->error("Unable to load specified eye model {}", sModel);
        return 1;
    }

    auto image_ = drishti::sdk::cvToDrishti<cv::Vec3b, drishti::sdk::Vec3b>(image);
    drishti::sdk::Eye eye;
    segmenter(image_, eye, isRight);

    cv::Mat1b mask(image.size(), 0);
    auto mask_ = drishti::sdk::cvToDrishti<uint8_t, uint8_t>(mask);

    const int maskKind = static_cast<int>(drishti::sdk::kScleraRegion) | static_cast<int>(drishti::sdk::kIrisRegion);
    drishti::sdk::createMask(mask_, eye, maskKind);

    // Output mask image:
    cv::imwrite(sOutput + "/mask.png", mask);

    // Output model parameters:
    std::string parameters = sOutput + "/model.json";

    std::ofstream ofs(parameters);    
    if (ofs)
    {
        ofs << drishti::sdk::EyeOStream(eye, drishti::sdk::EyeOStream::JSON);
    }
    else
    {
        logger->error("Unable to write file: {}", parameters);
    }

    return 0;
}

#if !defined(DRISHTI_TEST_BUILD_TESTS)
int main(int argc, char **argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch(const std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}
#endif

static std::shared_ptr<spdlog::logger> createLogger(const char *name)
{
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());
#if defined(__ANDROID__)
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink>());
#endif
    auto logger = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
    spdlog::register_logger(logger);
    spdlog::set_pattern("[%H:%M:%S.%e | thread:%t | %n | %l]: %v");
    return logger;
}
