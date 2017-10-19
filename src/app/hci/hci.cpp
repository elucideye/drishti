/*! -*-c++-*-
  @file   hci.cpp
  @author David Hirvonen
  @brief  Face and eye tracking, optical flow, corner detection, etc.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/core/Semaphore.h"
#include "drishti/hci/FaceFinderPainter.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/graphics/swizzle.h" // ogles_gpgpu...
#include "drishti/face/FaceDetectorFactoryJson.h"

#include "videoio/VideoSourceCV.h"
#include "videoio/VideoSinkCV.h"

#include "aglet/GLContext.h"

// Package includes:
#include "cxxopts.hpp"
#include "ogles_gpgpu/common/proc/disp.h"

#include <spdlog/fmt/ostr.h>

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

#include <opencv2/highgui.hpp>

static void* void_ptr(const cv::Mat& image)
{
    return const_cast<void*>(image.ptr<void>());
}

using LoggerPtr = std::shared_ptr<spdlog::logger>;

static bool checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description);

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-hci");

    // ############################
    // ### Command line parsing ###
    // ############################

    bool doWindow = false;
    bool doMovie = false;

    std::string sInput, sOutput;

    float cascCal = 0.f;
    float scale = 1.f;
    float fx = 0.f;

    // Create FaceDetectorFactory (default file based):
    std::string sFactory;
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();

    cxxopts::Options options("drishti-hci", "Command line interface for video sequence FaceFinder processing.");
    
    float minZ = 0.1f, maxZ = 2.f;

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))

#if !defined(DRISHTI_IS_MOBILE)
        ("w,window", "Create a display window", cxxopts::value<bool>(doWindow))
#endif
    
        // Generate a quicktime movie:
        ("m,movie", "Output quicktime movie", cxxopts::value<bool>(doMovie))
    
        // Detection and regression parameters:
        ("c,calibration", "Cascade calibration", cxxopts::value<float>(cascCal))
        ("s,scale", "Scale term for detection->regression mapping", cxxopts::value<float>(scale))
        ("f,focal-length", "Focal length in pixels",cxxopts::value<float>(fx))
        ("min", "Nearest distance in meters", cxxopts::value<float>(minZ))
        ("max", "Farthest distance in meters", cxxopts::value<float>(maxZ))
    
        // Clasifier and regressor models:
        ("D,detector", "Face detector model", cxxopts::value<std::string>(factory->sFaceDetector))
        ("M,mean", "Face detector mean", cxxopts::value<std::string>(factory->sFaceDetectorMean))
        ("R,regressor", "Face regressor", cxxopts::value<std::string>(factory->sFaceRegressor))
        ("E,eye", "Eye model", cxxopts::value<std::string>(factory->sEyeRegressor))

        // ... factory can be used instead of D,M,R,E
        ("F,factory", "Factory (json model zoo)", cxxopts::value<std::string>(sFactory))
    
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({ "" }) << std::endl;
        return 0;
    }

    // ############################################
    // ### Command line argument error checking ###
    // ############################################

    // ### Directory
    if (sOutput.empty())
    {
        logger->error("Must specify output directory");
        return 1;
    }

    if (drishti::cli::directory::exists(sOutput, ".drishti-hci"))
    {
        std::string filename = sOutput + "/.drishti-hci";
        remove(filename.c_str());
    }
    else
    {
        logger->error("Specified directory {} does not exist or is not writeable", sOutput);
        return 1;
    }

    // ### Input
    if (sInput.empty())
    {
        logger->error("Must specify input image or list of images");
        return 1;
    }
    if (!sInput.find(".test") && !drishti::cli::file::exists(sInput))
    {
        logger->error("Specified input file does not exist or is not readable");
        return 1;
    }

    if(!sFactory.empty())
    {
        factory = std::make_shared<drishti::face::FaceDetectorFactoryJson>(sFactory);
    }
    
    // Check for valid models
    std::vector<std::pair<std::string, std::string>> config{
        { factory->sFaceDetector, "face-detector" },
        { factory->sFaceDetectorMean, "face-detector-mean" },
        { factory->sFaceRegressor, "face-regressor" },
        { factory->sEyeRegressor, "eye-regressor" }
    };

    for (const auto& c : config)
    {
        if (checkModel(logger, c.first, c.second))
        {
            return 1;
        }
    }

    // !!! BUG: The current event queue and the main event queue are not the same.
    // Events will not be handled correctly. This is probably because _TSGetMainThread
    // was called for the first time off the main thread.

    // NOTE: We can create the OpenGL context prior to AVFoundation use as a workaround
    auto opengl = aglet::GLContext::create(aglet::GLContext::kAuto, doWindow ? "hci" : "", 640, 480);

    auto video = drishti::videoio::VideoSourceCV::create(sInput);
    video->setOutputFormat(drishti::videoio::VideoSourceCV::ARGB); // be explicit, fail on error

    // Retrieve first frame to configure sensor parameters:
    std::size_t counter = 0;
    auto frame = (*video)(counter);
    if (frame.image.empty())
    {
        logger->info("No frames available in video");
        return -1;
    }
    
    if(maxZ < minZ)
    {
        logger->error("max distance must be > min distance");
        return -1;
    }

    opengl->resize(frame.cols(), frame.rows());

    // Create configuration:
    drishti::hci::FaceFinder::Settings settings;
    settings.logger = drishti::core::Logger::create("test-drishti-hci");
    settings.outputOrientation = 0;
    settings.frameDelay = 2;
    settings.doLandmarks = true;
    settings.doFlow = false;
    settings.doBlobs = false;
    settings.threads = std::make_shared<tp::ThreadPool<>>();
    settings.outputOrientation = 0;
    settings.faceFinderInterval = 0.f;
    settings.regressorCropScale = scale;
    settings.acfCalibration = cascCal;
    settings.renderFaces = true;      // *** rendering ***
    settings.renderPupils = true;     // *** rendering ***
    settings.renderCorners = false;   // *** rendering ***
    settings.minDetectionDistance = minZ;
    settings.maxDetectionDistance = maxZ;
    { // Create a sensor specification
        if(fx == 0.f)
        {
            fx = frame.image.cols; // use a sensible default guess
        }
        const cv::Point2f p(frame.image.cols / 2, frame.image.rows / 2);
        drishti::sensor::SensorModel::Intrinsic params(p, fx, frame.image.size());
        settings.sensor = std::make_shared<drishti::sensor::SensorModel>(params);
    }

    (*opengl)(); // active context

    // Allocate the detector:
    auto detector = drishti::hci::FaceFinderPainter::create(factory, settings, nullptr);
    detector->setLetterboxHeight(1.0);         // *** rendering ***
    detector->setShowMotionAxes(false);        // *** rendering ***
    detector->setShowDetectionScales(false);   // *** rendering ***

    ogles_gpgpu::VideoSource source;
    ogles_gpgpu::SwizzleProc swizzle(ogles_gpgpu::SwizzleProc::kSwizzleGRAB); // kSwizzleRGBA
    source.set(&swizzle);

    std::string filename = sOutput + "/movie.mov";
    if (drishti::cli::file::exists(filename))
    {
        remove(filename.c_str());
    }

    std::shared_ptr<drishti::videoio::VideoSinkCV> sink;
    if (doMovie)
    {
        sink = drishti::videoio::VideoSinkCV::create(filename, ".mov");
        if (sink)
        {
            sink->setProperties({ frame.cols(), frame.rows() });
            sink->begin();
        }
    }

    std::shared_ptr<ogles_gpgpu::Disp> display;
    if (doWindow && opengl->hasDisplay())
    {
        display = std::make_shared<ogles_gpgpu::Disp>();
        display->init(frame.image.cols, frame.image.rows, TEXTURE_FORMAT);
        display->setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
    }

    std::function<bool(void)> render = [&]() {
        frame = (*video)(counter++);
        if (frame.image.empty())
        {
            return false;
        }
        if (frame.image.channels() == 3)
        {
            cv::cvtColor(frame.image, frame.image, cv::COLOR_BGR2BGRA);
        }

        CV_Assert(frame.image.channels() == 4);

        logger->info("{}", cv::mean(frame.image));

        // Perform texture swizzling:
        source({ { frame.cols(), frame.rows() }, void_ptr(frame.image), true, 0, TEXTURE_FORMAT });
        auto texture0 = swizzle.getOutputTexId();
        auto texture1 = (*detector)({ { frame.cols(), frame.rows() }, nullptr, false, texture0, TEXTURE_FORMAT });

        // Convert to texture as one of GL_BGRA or GL_RGBA
        if (display)
        {
            auto& geometry = opengl->getGeometry();
            display->setOffset(geometry.tx, geometry.ty);
            display->setDisplayResolution(geometry.sx, geometry.sy);
            display->useTexture(texture1);
            display->render(0);
        }

        if (sink && sink->good())
        {
            // clang-format off
            drishti::hci::FaceFinderPainter::FrameDelegate delegate = [&](const cv::Mat& image)
            {
                (*sink)(image);
            };
            // clang-format on
            detector->getOutputPixels(delegate);
        }

        return true;
    };

    (*opengl)(render);

    if (sink)
    {
        drishti::core::Semaphore s(0);
        sink->end([&] { s.signal(); });
        s.wait();
    }
    return 0;
}

/*
// This has been replaced by drishti_test_lib main()
// for cross-platform console app, but is left to 
// support standalone use as needed.
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

    return 0;
}
*/

// utility:

static bool
checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description)
{
    if (sModel.empty())
    {
        logger->error("Must specify valid model {}", sModel);
        return 1;
    }
    if (!drishti::cli::file::exists(sModel))
    {
        logger->error("Specified file {} does not exist or is not readable", sModel);
        return 1;
    }
    return 0;
}
