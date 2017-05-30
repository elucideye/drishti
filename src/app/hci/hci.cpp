/*!
  @file   hci.cpp
  @author David Hirvonen
  @brief  Face and eye tracking, optical flow, corner detection, etc.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/hci/FaceFinderPainter.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/graphics/swizzle.h" // ogles_gpgpu...

#include "videoio/VideoSourceCV.h"
#include "videoio/VideoSinkCV.h"
#include "GLWindow.h"

// Package includes:
#include "cxxopts.hpp"
#include "ogles_gpgpu/common/proc/disp.h"

// AVFoundation can trigger an error in GLFW if it runs first.
// We make sure to open a temporary windows w/ glfw based
// imshow as a workaround for the problem

/*
 !!! BUG: The current event queue and the main event queue are not the same.
 Events will not be handled correctly.
 This is probably because _TSGetMainThread was called for the first time off the main thread.
*/

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h" // needed for AVFoundation
#endif
// clang-format on

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

#include <opencv2/highgui.hpp>

static void * void_ptr(const cv::Mat &image)
{
    return const_cast<void *>(image.ptr<void>());
}

using LoggerPtr = std::shared_ptr<spdlog::logger>;

static void
wait(std::shared_ptr<drishti::videoio::VideoSinkCV> &video);

static void
initWindow(const std::string &name);

static bool
checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description);

int drishti_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-hci");

    // ############################
    // ### Command line parsing ###
    // ############################
    
    bool doMovie = false;

    std::string sInput, sOutput;

    // Full set of models must be specified:
    std::string sFaceDetector;
    std::string sFaceDetectorMean;
    std::string sFaceRegressor;
    std::string sEyeRegressor;
    
    float cascCal = 0.f;
    float scale = 1.f;
    
    cxxopts::Options options("drishti-hci", "Command line interface for video sequence FaceFinder processing.");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
    
        // Generate a quicktime movie:
        ("m,movie", "Output quicktime movie", cxxopts::value<bool>(doMovie))
    
        // Detection and regression parameters:
        ("c,calibration", "Cascade calibration", cxxopts::value<float>(cascCal))
        ("s,scale", "Scale term for detection->regression mapping", cxxopts::value<float>(scale))
    
        // Clasifier and regressor models:
        ("D,detector", "Face detector model", cxxopts::value<std::string>(sFaceDetector))
        ("M,mean", "Face detector mean", cxxopts::value<std::string>(sFaceDetectorMean))
        ("R,regressor", "Face regressor", cxxopts::value<std::string>(sFaceRegressor))
        ("E,eye", "Eye model", cxxopts::value<std::string>(sEyeRegressor))
    
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
        logger->error() << "Must specify output directory";
        return 1;
    }

    if (drishti::cli::directory::exists(sOutput, ".drishti-hci"))
    {
        std::string filename = sOutput + "/.drishti-hci";
        remove(filename.c_str());
    }
    else
    {
        logger->error() << "Specified directory " << sOutput << " does not exist or is not writeable";
        return 1;
    }

    // ### Input
    if (sInput.empty())
    {
        logger->error() << "Must specify input image or list of images";
        return 1;
    }
    if (!drishti::cli::file::exists(sInput))
    {
        logger->error() << "Specified input file does not exist or is not readable";
        return 1;
    }

    // Check for valid models
    std::vector<std::pair<std::string, std::string>> config{
        { sFaceDetector, "face-detector" },
        { sFaceDetectorMean, "face-detector-mean" },
        { sFaceRegressor, "face-regressor" },
        { sEyeRegressor, "eye-regressor" }
    };

    for (const auto& c : config)
    {
        if (checkModel(logger, c.first, c.second))
        {
            return 1;
        }
    }

#if defined(DRISHTI_USE_IMSHOW)
    initWindow("face");
#endif
    
    auto video = drishti::videoio::VideoSourceCV::create(sInput);
    video->setOutputFormat(drishti::videoio::VideoSourceCV::ARGB); // be explicit, fail on error

    // Retrieve first frame to configure sensor parameters:
    std::size_t counter = 0;
    auto frame = (*video)(counter);
    if(frame.image.empty())
    {
        logger->info() << "No frames available in video";
        return -1;
    }

    // Create FaceDetectorFactory (default file based):
    std::shared_ptr<drishti::face::FaceDetectorFactory> factory;
    factory = std::make_shared<drishti::face::FaceDetectorFactory>();
    factory->sFaceDetector = sFaceDetector;
    factory->sFaceRegressors = { sFaceRegressor };
    factory->sEyeRegressor = sEyeRegressor;
    factory->sFaceDetectorMean = sFaceDetectorMean;
    
    // Create configuration:
    drishti::hci::FaceFinder::Settings settings;
    settings.logger = drishti::core::Logger::create("test-drishti-hci");
    settings.outputOrientation = 0;
    settings.frameDelay = 2;
    settings.doLandmarks = true;
    settings.doFlow = true;
    settings.doBlobs = false;
    settings.threads = std::make_shared<tp::ThreadPool<>>();
    settings.outputOrientation = 0;
    settings.faceFinderInterval = 0.f;
    settings.regressorCropScale = scale;
    settings.acfCalibration = cascCal;
    
    settings.renderFaces = true;
    settings.renderPupils = true;
    settings.renderCorners = false;

    { // Create a sensor specification
        const float fx = frame.image.cols;
        const cv::Point2f p(frame.image.cols / 2, frame.image.rows / 2);
        drishti::sensor::SensorModel::Intrinsic params(p, fx, frame.image.size());
        settings.sensor = std::make_shared<drishti::sensor::SensorModel>(params);
    }

    // Create an OpenGL context:
    GLWindow window("player", frame.cols(), frame.rows());

    // Allocate the detector:
    auto detector = drishti::hci::FaceFinderPainter::create(factory, settings, nullptr);
    detector->setLetterboxHeight(1.0); // show full video for offline sequences
    detector->setShowMotionAxes(false);
    detector->setShowDetectionScales(false);
    
    ogles_gpgpu::VideoSource source;
    ogles_gpgpu::SwizzleProc swizzle(ogles_gpgpu::SwizzleProc::kSwizzleGRAB);
    source.set(&swizzle);
    
    ogles_gpgpu::Disp disp;
    disp.init(frame.image.cols, frame.image.rows, TEXTURE_FORMAT);
    disp.setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
    
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
            sink->setProperties({frame.cols(), frame.rows()});
            sink->begin();
        }
    }
    
    std::function<bool(void)> render = [&]()
    {
        disp.setDisplayResolution(GLWindow::impl.sx, GLWindow::impl.sy);
        frame = (*video)(counter);
        if(frame.image.empty())
        {
            return false;
        }
        
        // Perform texture swizzling:
        source({{frame.cols(),frame.rows()}, void_ptr(frame.image), true, 0, TEXTURE_FORMAT});
        
        // Convert to texture as one of GL_BGRA or GL_RGBA
        auto texture0 = swizzle.getOutputTexId();
        auto texture1 = (*detector)({{frame.cols(),frame.rows()}, nullptr, false, texture0, TEXTURE_FORMAT});
        disp.useTexture(texture1);
        disp.render(0);
        
        if(sink && sink->good())
        {
            drishti::hci::FaceFinderPainter::FrameDelegate delegate = [&](const cv::Mat &image)
            {
                (*sink)(image);
            };
            detector->getOutputPixels(delegate);
        }
        return true;
    };

    window(render);

    if(sink)
    {
        wait(sink);
    }
    return 0;
}

int main(int argc, char** argv)
{
    try
    {
        return drishti_main(argc, argv);
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

// utility:

static void
wait(std::shared_ptr<drishti::videoio::VideoSinkCV> &sink)
{
    bool isFinished = false;
    std::mutex mutex;
    std::unique_lock<std::mutex> lock(mutex);
    std::condition_variable cv;
    std::function<void()> wait = [&]{ isFinished = true; };
    if(sink->end(wait))
    {
        cv.wait(lock, [&] { return isFinished; });
    }
}

static bool
checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description)
{
    if (sModel.empty())
    {
        logger->error() << "Must specify valid model " << sModel;
        return 1;
    }
    if (!drishti::cli::file::exists(sModel))
    {
        logger->error() << "Specified model file does not exist or is not readable";
        return 1;
    }
    return 0;
}

#if defined(DRISHTI_USE_IMSHOW)
static void initWindow(const std::string &name)
{
    // Hack/workaround needed for continuous preview in current imshow lib
    cv::Mat canvas(240, 320, CV_8UC3, cv::Scalar(0, 255, 0));
    cv::putText(canvas, "GLFW Fix", {canvas.cols/4, canvas.rows/2}, CV_FONT_HERSHEY_PLAIN, 2.0, {0,0,0});
    glfw::imshow(name.c_str(), canvas);
    glfw::waitKey(1);
    glfw::destroyWindow(name.c_str());
}
#endif
