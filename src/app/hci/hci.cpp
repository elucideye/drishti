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
#include "drishti/core/Logger.h"
#include "drishti/hci/FaceFinderPainter.h"
#include "drishti/hci/FaceMonitor.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/face/FaceDetectorFactoryJson.h"
#include "drishti/core/drishti_string_hash.h"

#include "ogles_gpgpu/common/proc/swizzle.h"

#include "videoio/VideoSourceCV.h"
#include "videoio/VideoSinkCV.h"

#include "aglet/GLContext.h"

// Package includes:
#include "cxxopts.hpp"

#include "ogles_gpgpu/common/proc/disp.h"
#include "ogles_gpgpu/common/proc/swizzle.h"

#include <spdlog/fmt/ostr.h>

using string_hash::operator"" _hash;

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

#define DRISHTI_HCI_USE_CACHE 0 // simulate speed of real video

#include <opencv2/highgui.hpp>

using LoggerPtr = std::shared_ptr<spdlog::logger>;

static void* void_ptr(const cv::Mat& image)
{
    return const_cast<void*>(image.ptr<void>());
}

static bool checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description);
static ogles_gpgpu::SwizzleProc::SwizzleKind getSwizzleKind(const std::string &sSwizzle);

// Simple FaceMonitor class to report face detection results over time.
struct FaceMonitorLogger : public drishti::hci::FaceMonitor
{
    FaceMonitorLogger(std::shared_ptr<spdlog::logger> &logger, int history)
        : history(history)
        , m_logger(logger)
    {
    }
    
    /**
     * A user defined virtual method callback that should report the number
     * of frames that should be captured from teh FIFO buffer based on the
     * reported face location.
     * @param faces a vector of faces for the current frame
     * @param timestmap the acquisition timestamp for the frame
     * @return a frame request for the last n frames with requested image formats
     */
    virtual Request request(const Faces& faces, const TimePoint& timeStamp, std::uint32_t texture)
    {
        cv::Point3f xyz = faces.size() ? (*faces.front().eyesCenter) : cv::Point3f();
        m_logger->info("SimpleFaceMonitor: Found {} faces {}", faces.size(), xyz);
        
        // clang-format off
        return
        {
            history, // histtory
            true,    // getImage
            true,    // getTexture
            false,   // getFrames (for high-res image this can be expensive)
            true     // getEyes
        };
        // clang-format on
    }
    
    /**
     * A user defined virtual method callback that will be called with a
     * a populated vector of FaceImage objects for the last N frames, where
     * N is the number of frames requested in the preceding request callback.
     * @param frames A vector containing the last N consecutive FaceImage objects
     * @param isInitialized Return true if the FIFO buffer is fully initialized.
     */
    virtual void grab(const std::vector<FaceImage>& frames, bool isInitialized)
    {
        int faces = 0, eyes = 0; // full images, and eye images
        for(const auto &f : frames)
        {
            if(!f.eyes.image.empty())
            {
                eyes++;
            }
            if(!f.image.image.empty())
            {
                faces++;
            }
        }

        m_logger->info("SimpleFaceMonitor[{}]: Frames={}; faces={} eyes={}", count++, frames.size(), faces, eyes);
    }
    
    std::size_t count = 0;
    int history;
    std::shared_ptr<spdlog::logger> m_logger;
};

// Add a simple timer to report frame rate.  For any offline VideoSource type
// the processing will almost certainly be IO limited.  As a simple workaround
// you can define
//
// #define DRISHTI_HCI_USE_CACHE 1
//
// For videos that will fit in memory and it will cache the frames in memory.
// The reported times will be closer to what one would see with a live video
// capture.
struct SimpleTimer
{
    int frames = 0;
    std::chrono::high_resolution_clock::time_point tic;
    void reset()
    {
        frames = 0;
        tic = std::chrono::high_resolution_clock::now();
    }
    SimpleTimer& operator++()
    {
        frames++;
        return (*this);
    }
    float fps() const
    {
        const auto toc = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration<double>(toc - tic).count();
        return static_cast<float>(frames / elapsed);
    }
};

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
    bool doDebug = false;
    bool doCpu = false;
    int loops = 0;
    
    std::string sInput, sOutput, sSwizzle = "rgba";

    float resolution = 1.f;
    float fx = 0.f;
    
    // Create FaceDetectorFactory (default file based):
    std::string sFactory;
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();

    cxxopts::Options options("drishti-hci", "Command line interface for video sequence FaceFinder processing.");

    float minZ = 0.1f, maxZ = 2.f;
    
    drishti::hci::FaceFinder::Settings settings;

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))

        ("swizzle", "Swizzle channel operation", cxxopts::value<std::string>(sSwizzle))
    
#if !defined(DRISHTI_IS_MOBILE)
        ("w,window", "Create a display window", cxxopts::value<bool>(doWindow))
        ("r,resolution", "Display resolution (scale factory)", cxxopts::value<float>(resolution))
        ("debug", "Provide debugging annotations", cxxopts::value<bool>(doDebug))
#endif
        ("l,loops", "Loop the input video", cxxopts::value<int>(loops))
    
        // Generate a quicktime movie:
        ("m,movie", "Output quicktime movie", cxxopts::value<bool>(doMovie))

        // ::::::::::::::::::::::::::::::::::::::::::
        // ::: drishti::hci::Facefinder::Settings :::
        // ::::::::::::::::::::::::::::::::::::::::::
    
        ("c,calibration", "Cascade calibration", cxxopts::value<float>(settings.acfCalibration))
        ("s,scale", "Scale term for detection->regression mapping", cxxopts::value<float>(settings.regressorCropScale))
        ("min", "Nearest distance in meters", cxxopts::value<float>(settings.minDetectionDistance))
        ("max", "Farthest distance in meters", cxxopts::value<float>(settings.maxDetectionDistance))
        ("min-track-hits", "Minimum consecutive detections to start a trck", cxxopts::value<std::size_t>(settings.minTrackHits))
        ("max-track-misses", "Minimum consecutive detections to start a trck", cxxopts::value<std::size_t>(settings.maxTrackMisses))
        ("f,focal-length", "Focal length in pixels",cxxopts::value<float>(fx))
        ("cpu", "Force CPU ACF processing", cxxopts::value<bool>(doCpu))
    
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
    
    if (options.count("focal-length") != 1)
    {
        logger->error("You must specify the focal-length (in pixels)");
        return 1;
    }

    if (!sFactory.empty())
    {
        factory = std::make_shared<drishti::face::FaceDetectorFactoryJson>(sFactory);
    }
    
    if (maxZ < minZ)
    {
        logger->error("max distance must be > min distance");
        return -1;
    }

    // Check for valid models
    // clang-format off
    std::vector<std::pair<std::string, std::string>> config
    {
        { factory->sFaceDetector, "face-detector" },
        { factory->sFaceDetectorMean, "face-detector-mean" },
        { factory->sFaceRegressor, "face-regressor" },
        { factory->sEyeRegressor, "eye-regressor" }
    };
        // clang-format on

    for (const auto& c : config)
    {
        if (checkModel(logger, c.first, c.second))
        {
            return 1;
        }
    }

    // In some glfw + avfoundation + os x combinations we can see the following system
    // error.  This may be behind us now!
    // ~~~~~
    // !!! BUG: The current event queue and the main event queue are not the same.
    // Events will not be handled correctly. This is probably because _TSGetMainThread
    // was called for the first time off the main thread.

    // NOTE: We can create the OpenGL context prior to AVFoundation use as a workaround
    auto opengl = aglet::GLContext::create(aglet::GLContext::kAuto, doWindow ? "hci" : "", 640, 480);
#if defined(_WIN32) || defined(_WIN64)
    CV_Assert(!glewInit());
#endif

    auto video = drishti::videoio::VideoSourceCV::create(sInput);
    video->setOutputFormat(drishti::videoio::VideoSourceCV::ARGB); // be explicit, fail on error

    // Retrieve first frame to configure sensor parameters:
    std::size_t counter = 0;
    auto frame = (*video)(counter);
    const cv::Size frameSize = frame.image.size();

    if (frame.image.empty())
    {
        logger->info("No frames available in video");
        return -1;
    }

    cv::Size windowSize = cv::Size2f(frameSize) * resolution;
    opengl->resize(windowSize.width, windowSize.height);

    // Create configuration:
    settings.logger = drishti::core::Logger::create("test-drishti-hci");
    settings.outputOrientation = 0;
    settings.frameDelay = 2;
    settings.doLandmarks = true;
    settings.doFlow = false;
    settings.doBlobs = false;
    settings.threads = std::make_shared<tp::ThreadPool<>>();
    settings.outputOrientation = 0;
    settings.faceFinderInterval = 0.f;
    settings.renderFaces = true;          // *** rendering ***
    settings.renderPupils = true;         // *** rendering ***
    settings.renderCorners = false;       // *** rendering ***
    settings.renderEyesWidthRatio = 0.25f * opengl->getGeometry().sx; // *** rendering ***
    settings.doSingleFace = true;
    settings.doOptimizedPipeline = !doCpu;
    
    // The following parameters are set directly through the command line parser:
    //
    //  settings.minDetectionDistance = ...;
    //  settings.maxDetectionDistance = ...;
    //  settings.minTrackHits = ...;
    //  settings.maxTrackMisses = ...;
    //  settings.regressorCropScale = ...;
    //  settings.acfCalibration = ...;

    { // Add intrinsic camera parameters:
        const cv::Point2f p(frame.image.cols / 2, frame.image.rows / 2);
        drishti::sensor::SensorModel::Intrinsic params(p, fx, frame.image.size());
        settings.sensor = std::make_shared<drishti::sensor::SensorModel>(params);
    }
    
    // One can customize the simple tracking step (data association)
    // by specifying constraints for the minimum # of hits required
    // to create a new track *and* the number of misses before the
    // track will die.
    //
    // For example, the following settings will start a new track
    // for every new detection (that is not mapped to an existing track)
    // and it will kill each track on any frame where there is no
    // new detection assigned to the track.
    //
    //   settings.minTrackHits = 0;
    //   settings.maxTrackMisses = 1;
    //
    // A more robust configuration could be achieved with the following
    // entries.  In this configuration a new track is not "born" until it
    // receives three consecutive detection assignments.  Similarly, it
    // is not terminated unless there are no correpsondending detection
    // assignements for 3 consecutive frames.  You can use
    //
    //   settings.minTrackHits = 3;
    //   settings.maxTrackMisses = 3;
    
    (*opengl)(); // activate context

    // Allocate the detector and configure the display properties
    auto detector = drishti::hci::FaceFinderPainter::create(factory, settings, nullptr);
    detector->setLetterboxHeight(1.0);         // *** rendering ***
    detector->setShowMotionAxes(doDebug);      // *** rendering ***
    detector->setShowDetectionScales(doDebug); // *** rendering ***
    detector->setDoCpuAcf(doCpu);
    
    // Instantiate and register a samle FaceMonitor class to log tracking results
    // over time.  Here we setup a sample callback that will request just the
    // last frame N=1 at each step, but we can set this up to request a buffer
    // of frames with something like N=3.
    FaceMonitorLogger faceMonitor(logger, 1);
    detector->registerFaceMonitorCallback(&faceMonitor);
    
    // Allocate an input video source for feeding texture or image buffers
    // into the gpgpu pipeline.
    ogles_gpgpu::VideoSource source;
    ogles_gpgpu::SwizzleProc swizzle(getSwizzleKind(sSwizzle));
    source.set(&swizzle);

    // Provide a default quicktime movie name for logging on Apple platforms.
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

    // Instantiate an ogles_gpgpu display class that will draw to the
    // default texture (0) which will be managed by aglet (typically glfw)
    std::shared_ptr<ogles_gpgpu::Disp> display;
    if (doWindow && opengl->hasDisplay())
    {
        display = std::make_shared<ogles_gpgpu::Disp>();
        display->init(frame.image.cols, frame.image.rows, TEXTURE_FORMAT);
        display->setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
    }

    SimpleTimer timer;
    
#if DRISHTI_HCI_USE_CACHE
    std::map<int, drishti::videoio::VideoSourceCV::Frame> cache;
#endif
    
    int loopCount = 0;
    std::function<bool(void)> render = [&]()
    {
#if DRISHTI_HCI_USE_CACHE
        if(cache.find(counter) != cache.end())
        {
            frame = cache[counter];
        }
        else
#endif
        {
            frame = (*video)(counter);

#if DRISHTI_HCI_USE_CACHE
            cache[counter] = frame; // cache it
#endif            
            timer.reset(); // reset timer
        }
        
        logger->info("fps: {}", (++timer).fps());
        
        counter++;
        if (frame.image.empty())
        {
            logger->info("Frame {} is empty, skipping ...", counter);
            
            if(loopCount < loops)
            {
                counter = 0;
                loopCount++;
                return true;
            }
            
            return false;
        }
        
        if (frame.image.size() != frameSize)
        {
            logger->info("Frame size has changed, skipping ...", frameSize);
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
            display->setDisplayResolution(geometry.sx * resolution, geometry.sy * resolution);
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

// utility:

static bool checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description)
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

static ogles_gpgpu::SwizzleProc::SwizzleKind getSwizzleKind(const std::string &sSwizzle)
{
    switch (string_hash::hash(sSwizzle))
    {
        case "rgba"_hash: return ogles_gpgpu::SwizzleProc::kSwizzleRGBA; break;
        case "bgra"_hash: return ogles_gpgpu::SwizzleProc::kSwizzleBGRA; break;
        case "argb"_hash: return ogles_gpgpu::SwizzleProc::kSwizzleARGB; break;
        case "abgr"_hash: return ogles_gpgpu::SwizzleProc::kSwizzleABGR; break;
        case "grab"_hash: return ogles_gpgpu::SwizzleProc::kSwizzleGRAB; break;
        default: throw std::runtime_error("Unsupported type specified in" + sSwizzle);
    }
}
