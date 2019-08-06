/*!
  @file   face.cpp
  @brief  Test the drishti face tracking API.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

  drishti-face-test \
    --input=0 \
    --models=${SOME_PATH_VAR}/drishti-assets/drishti_assets_big.json \
    --config=${DHT_REPO}/config/logitech_c615.json \
    --preview

*/

// Need std:: extensions for android targets
#if !defined(DRISHTI_HAVE_TO_STRING)
#    include "stdlib_string.h"
#endif

#include <spdlog/spdlog.h> // for portable logging
#include <spdlog/sinks/stdout_sinks.h>

// clang-format off
#if defined(__ANDROID__) || defined(ANDROID)
#  include <spdlog/sinks/android_sink.h>
#endif
// clang-format on

#include <spdlog/fmt/ostr.h>

#include <facefilter/renderer/FaceTrackerFactoryJson.h>
#include <facefilter/renderer/Context.h> // facefilter::Application
#include "VideoCaptureList.h"

#include "drishti/drishti_sdk.hpp" // for version from public SDK

#include <opencv2/core.hpp>    // for cv::Mat
#include <opencv2/imgproc.hpp> // for cv::cvtColor()
#include <opencv2/highgui.hpp> // for cv::imread()

#include <aglet/GLContext.h> // for portable opengl context

#include <cxxopts.hpp> // for CLI parsing

#include <fstream>
#include <istream>
#include <sstream>
#include <iomanip>

#include <facefilter/Exception.hpp>

#ifdef ANDROID
#    define DFLT_TEXTURE_FORMAT GL_RGBA
#else
#    define DFLT_TEXTURE_FORMAT GL_BGRA
#endif

struct Params
{
    int videoWidth = 0;
    int videoHeight = 0;
    float focalLength = 0.f;
};

static void from_json(const std::string& filename, Params& params);

// avoid localeconv error w/ nlohmann::json on older android build

#if defined(DRISHTI_HAVE_LOCALECONV)
static void to_json(const std::string& filename, const Params& params);
#endif

using FaceResources = drishti::sdk::FaceTracker::Resources;
static std::shared_ptr<cv::VideoCapture> create(const std::string& filename);
static std::shared_ptr<spdlog::logger> createLogger(const char* name);
static cv::Size getSize(const cv::VideoCapture& video);

int main(int argc, char** argv)
{
    auto logger = createLogger("drishti-face-test");

    const auto argumentCount = argc;

    bool doPreview = false;
    std::string sInput, sModels, sConfig;

#if defined(DRISHTI_HAVE_LOCALECONV)
    std::string sBoilerplate;
#endif

    bool doVersion = false;

    Params params;

    cxxopts::Options options("drishti-face-test", "Command line interface for face model fitting");

    // clang-format off
    options.add_options()
        // input/output:
        ("i,input", "Input image", cxxopts::value<std::string>(sInput))
        ("m,models", "Model factory configuration file (JSON)", cxxopts::value<std::string>(sModels))
        ("c,config", "Configuration file", cxxopts::value<std::string>(sConfig))
#if defined(DRISHTI_HAVE_LOCALECONV)
        ("boilerplate", "Dump boilerplate json file (then quit)", cxxopts::value<std::string>(sBoilerplate))
#endif
        // context parameters (configuration):
        ("focal-length", "focal length", cxxopts::value<float>(params.focalLength))

        // behavior:
        ("p,preview", "Preview window", cxxopts::value<bool>(doPreview))

        ("version", "Report library version", cxxopts::value<bool>(doVersion))
        ;
    // clang-format on

    auto parseResult = options.parse(argc, argv);
    if ((argumentCount <= 1) || parseResult.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return 0;
    }

    if (doVersion)
    {
        logger->info("Version: {}", DRISHTI_VERSION);
        return 0;
    }

    if (sInput.empty())
    {
        logger->error("Must specify input {}", sInput);
        return 1;
    }

    if (sModels.empty())
    {
        logger->error("Must specify models file {}", sModels);
        return 1;
    }

#if defined(DRISHTI_HAVE_LOCALECONV)
    if (!sBoilerplate.empty())
    {
        to_json(sBoilerplate, params);
        return 0;
    }
#endif

    // User must specify either:
    // (a) json file with complete parameter set
    // (b) focal length
    if (!sConfig.empty())
    {
        from_json(sConfig, params);
    }
    else
    {
        if (parseResult.count("focal-length") != 1)
        {
            logger->error("Must specify focal length (in pixels)");
            return 1;
        }
    }

    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    // Allocate a video source and get the video frame dimensions:
    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::

    std::shared_ptr<cv::VideoCapture> video = create(sInput);
    if (!(video && video->isOpened()))
    {
        logger->error("Failed to create video source for {}", sInput);
        return 1;
    }

    video->set(cv::CAP_PROP_FRAME_WIDTH, params.videoWidth);
    video->set(cv::CAP_PROP_FRAME_HEIGHT, params.videoHeight);

    cv::Size size = getSize(*video);
    if (size.area() == 0)
    {
        logger->error("Failed to read a frame from device {}", sInput);
        return 1;
    }

    if ((size.width != params.videoWidth) || (size.height != params.videoHeight))
    {
        logger->error(
            "Failed to read a video frame with requested dimensions, received {}x{} expected {}x{}",
            size.width,
            size.height,
            params.videoWidth,
            params.videoHeight);
        return 1;
    }

    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    // Create an OpenGL context (w/ optional window):
    // :::::::::::::::::::::::::::::::::::::::::::::::::::::::::::
    std::shared_ptr<aglet::GLContext> glContext;
    if (doPreview)
    {
        glContext = aglet::GLContext::create(aglet::GLContext::kAuto, "drishti-face-test", size.width, size.height);
    }
    else
    {
        glContext = aglet::GLContext::create(aglet::GLContext::kAuto);
    }

    if (!glContext)
    {
        logger->error("Failed to create OpenGL context");
        return 1;
    }

    (*glContext)();

    float resolution = 1.0f;
    const auto tic = std::chrono::high_resolution_clock::now();
    std::size_t index = 0;

    facefilter::Application app;
    app.loadAsset("drishti_assets", sModels.c_str());
    app.initDisplay(size.width, size.height);
    const int cameraRotation = 0; // TODO (???)
    app.initCamera(size.width, size.height, cameraRotation, params.focalLength);

    std::function<bool()> process = [&]() {
        cv::Mat image;
        (*video) >> image;

        if (image.empty())
        {
            logger->error("Unable to read image {}", sInput);
            return false;
        }

        //  cv::imwrite("c:/tmp/frame.png", image); // 1920 / fx =  26 / 20; fx = 1920 * 20/26

        if (size.area() && (size != image.size()))
        {
            logger->error("Frame dimensions must be consistent: {}{}", size.width, size.height);
        }

        if (image.channels() == 3)
        {
            cv::cvtColor(image, image, cv::COLOR_BGR2BGRA);
        }

        if (doPreview)
        { // Update window properties (if used):
            auto& win = glContext->getGeometry();
            app.setPreviewGeometry(win.tx, win.ty, win.sx, win.sy);
        }

        app.drawFrame(image.ptr(), true, DFLT_TEXTURE_FORMAT);

        { // Comnpute simple/global FPS
            const auto toc = std::chrono::high_resolution_clock::now();
            const double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(toc - tic).count();
            const double fps = static_cast<double>(index + 1) / elapsed;
            logger->info("Frame: {} fps = {}", index++, fps);
        }

        return true;
    };

    (*glContext)(process);

    app.destroy();

    return 0;
}

static std::shared_ptr<spdlog::logger> createLogger(const char* name)
{
    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());
#if defined(__ANDROID__) || defined(ANDROID)
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink_mt>());
#endif
    auto logger = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
    spdlog::register_logger(logger);
    spdlog::set_pattern("[%H:%M:%S.%e | thread:%t | %n | %l]: %v");
    return logger;
}

static std::shared_ptr<cv::VideoCapture> create(const std::string& filename)
{
    if (filename.find_first_not_of("0123456789") == std::string::npos)
    {
        auto ptr = std::make_shared<cv::VideoCapture>(std::stoi(filename));
        if (ptr && !ptr->isOpened())
        {
            ptr->open(0 + cv::CAP_ANY);
        }
        return ptr;
    }
    else if (filename.find(".txt") != std::string::npos)
    {
        return std::make_shared<VideoCaptureList>(filename);
    }
    else
    {
        return std::make_shared<cv::VideoCapture>(filename);
    }
}

static cv::Size getSize(const cv::VideoCapture& video)
{
    return {
        static_cast<int>(video.get(cv::CAP_PROP_FRAME_WIDTH)),
        static_cast<int>(video.get(cv::CAP_PROP_FRAME_HEIGHT))
    };
};

#include <nlohmann/json.hpp> // nlohman-json

static void from_json(const nlohmann::json& json, Params& params)
{
    params.videoWidth = json.at("videoWidth").get<float>();
    params.videoHeight = json.at("videoHeight").get<float>();
    params.focalLength = json.at("focalLength").get<float>();
}

static void from_json(const std::string& filename, Params& params)
{
    std::ifstream ifs(filename);
    if (!ifs)
    {
        throw facefilter::Exception(facefilter::Exception::FILE_OPEN);
    }

    nlohmann::json json;
    ifs >> json;
    from_json(json, params);
}

#if defined(DRISHTI_HAVE_LOCALECONV)
static void to_json(nlohmann::json& json, const Params& params)
{
    json = nlohmann::json{
        { "videoWidth", params.videoWidth },
        { "videoHeight", params.videoHeight },
        { "focalLength", params.focalLength },
    };
}

static void to_json(const std::string& filename, const Params& params)
{
    std::ofstream ofs(filename);
    if (!ofs)
    {
        throw facefilter::Exception(facefilter::Exception::FILE_OPEN);
    }

    nlohmann::json json;
    to_json(json, params);
    ofs << std::setw(4) << json;
}
#endif
