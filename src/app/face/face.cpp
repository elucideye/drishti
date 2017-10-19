/*! -*-c++-*-
  @file   face.cpp
  @author David Hirvonen
  @brief  Face detection and landmark estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/acf/ACF.h"
#include "drishti/core/LazyParallelResource.h"
#include "drishti/core/Line.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/scope_guard.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/face/FaceDetector.h"
#include "drishti/face/FaceDetectorFactoryJson.h"
#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/geometry/motion.h"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include "videoio/VideoSourceCV.h"

// Package includes:
#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <cereal/archives/json.hpp>

using LoggerPtr = std::shared_ptr<spdlog::logger>;

static cv::Mat
cropEyes(const cv::Mat& image, const drishti::face::FaceModel& face, const cv::Size& size, float scale, bool annotate);
static void initWindow(const std::string& name);
static bool writeAsJson(const std::string& filename, const std::vector<drishti::face::FaceModel>& faces);
static void drawObjects(cv::Mat& canvas, const std::vector<drishti::face::FaceModel>& faces);
static bool checkModel(LoggerPtr& logger, const std::string& sModel, const std::string& description);

// Resize input image to detection objects of minimum width
// given an object detection window size. i.e.,
//
// ACF windowSize = {48x48};
//
// To restrict object detection search to find objects of
// minWidth == 100 pixels, we would need to *downsample*
// the image by a factor of 48/100 (nearly 0.5x)

class Resizer
{
public:
    using PaddedImage = drishti::face::FaceDetector::PaddedImage;

    // Create:
    // 1) reduced+RGB+float+transposed low resolution image
    // 2) single channel padded grayscale image (higher resolution)
    Resizer(cv::Mat& image, const cv::Size& winSize, int width = -1)
    {
        if ((width >= 0) && !image.empty())
        {
            Sfd = static_cast<float>(winSize.width) / static_cast<float>(width);
            const int interpolation = (Sfd < 1.f) ? cv::INTER_AREA : cv::INTER_LINEAR;
            cv::resize(image, reduced, {}, Sfd, Sfd, interpolation);
        }
        else
        {
            reduced = image;
        }

        { // Create the high resolution padded image for regression:
            cv::Mat green;
            cv::extractChannel(image, green, 1);
            padded = PaddedImage(green, { { 0, 0 }, green.size() });
        }

        { // Create the lower resolution image for detection:
            cv::Mat It = reduced.t(), Itf;
            It.convertTo(Itf, CV_32FC3, 1.0f / 255.f);
            planar = MatP(Itf);
            Sdr = 1.f / Sfd;
        }

        // Store the transformation from the detection to the regression image:
        Hdr = cv::Matx33f(cv::Matx33f::diag({ Sdr, Sdr, 1.f }));

        // Compute and store the transformation from the regressor to full resolution:
        const float Srd = (1.f / Sdr), Sdf = (1.f / Sfd);
        Srf = Sdf * Srd;
        Hrf = cv::Matx33f::diag({ Srf, Srf, 1.f }); // regressor->full
    }

    // Faces will be avaialble at the regressor resolution:
    void operator()(std::vector<drishti::face::FaceModel>& faces) const
    {
        if (Srf != 1.f)
        {
            for (auto& f : faces)
            {
                f = Hrf * f;
            }
        }
    }

    const cv::Matx33f& getRegressorToFull() const { return Hrf; }
    const cv::Matx33f& getDetectorToRegressor() const { return Hdr; }

    const MatP& getPlanar() const { return planar; }
    MatP& getPlanar() { return planar; }

    const PaddedImage& getPadded() const { return padded; }
    PaddedImage& getPadded() { return padded; }

protected:
    float Sfd = 1.f; // scale: full to detection
    float Sdr = 1.f; // scale: detection to regression
    float Srf = 1.f;

    cv::Matx33f Hdr; // scale: detector to regressor
    cv::Matx33f Hrf; // scale: regressor to full

    cv::Mat reduced;
    MatP planar;
    PaddedImage padded;
};

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-acf");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput;
    int threads = -1;
    bool doEyes = false;
    bool doPause = false;
    bool doDisplay = false;
    bool doAnnotation = false;
    bool doPositiveOnly = false;
    float scale = 1.0;
    double cascCal = 0.0;
    int minWidth = -1; // minimum object width

    // Use factory as container for CLI inputs:
    std::string sFactory;
    auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();

    float minZ = 0.1f, maxZ = 1.f;
    
    cxxopts::Options options("drishti-acf", "Command line interface for ACF object detection (see Piotr's toolbox)");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
    
        // Detection parameters:
        ("l,min", "Minimum object width (lower bound)", cxxopts::value<int>(minWidth))
        ("c,calibration", "Cascade calibration", cxxopts::value<double>(cascCal))
        ("s,scale", "Scale term for detection->regression mapping", cxxopts::value<float>(scale))

        // Clasifier and regressor models:
        ("D,detector", "Face detector model", cxxopts::value<std::string>(factory->sFaceDetector))
        ("M,mean", "Face detector mean", cxxopts::value<std::string>(factory->sFaceDetectorMean))
        ("R,regressor", "Face regressor", cxxopts::value<std::string>(factory->sFaceRegressor))
        ("E,eye", "Eye model", cxxopts::value<std::string>(factory->sEyeRegressor))
    
        // ... factory can be used instead of D,M,R,E
        ("F,factory", "Factory (json model zoo)", cxxopts::value<std::string>(sFactory))
    
    
    
        // Output parameters:
        ("e,eyes", "Crop eyes", cxxopts::value<bool>(doEyes))
        ("a,annotate", "Create annotated images", cxxopts::value<bool>(doAnnotation))
        ("d,display", "Display window (single thread)", cxxopts::value<bool>(doDisplay))
        ("0,pause", "Pause display window", cxxopts::value<bool>(doPause))
        ("p,positive", "Limit output to positve examples", cxxopts::value<bool>(doPositiveOnly))
        ("t,threads", "Thread count", cxxopts::value<int>(threads))
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

    if (drishti::cli::directory::exists(sOutput, ".drishti-acf"))
    {
        std::string filename = sOutput + "/.drishti-acf";
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
    if (!drishti::cli::file::exists(sInput))
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

#if defined(DRISHTI_USE_IMSHOW)
    if (doDisplay)
    {
        initWindow("face");
    }
#endif

    auto video = drishti::videoio::VideoSourceCV::create(sInput);

    // Allocate resource manager:
    using FaceDetectorPtr = std::unique_ptr<drishti::face::FaceDetector>;
    drishti::core::LazyParallelResource<std::thread::id, FaceDetectorPtr> manager = [&]() {

        FaceDetectorPtr detector = drishti::core::make_unique<drishti::face::FaceDetector>(*factory);
        detector->setScaling(scale);
        if (detector)
        {
            // Cofigure parameters:
            detector->setDoNMS(true);
            detector->setDoNMSGlobal(true);

            auto acf = dynamic_cast<drishti::acf::Detector*>(detector->getDetector());
            if (acf && acf->good())
            {
                // Cascade threhsold adjustment:
                if (cascCal != 0.f)
                {
                    drishti::acf::Detector::Modify dflt;
                    dflt.cascThr = { "cascThr", -1.0 };
                    dflt.cascCal = { "cascCal", cascCal };
                    acf->acfModify(dflt);
                }
            }
            else
            {
                detector.release();
            }
        }

        return detector;
    };

    std::size_t total = 0;

    // Parallel loop:
    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        // Get thread specific segmenter lazily:
        auto& detector = manager[std::this_thread::get_id()];
        assert(detector);

        // Load current image:
        auto frame = (*video)(i);
        const auto& image = frame.image;

        if (!image.empty())
        {
            cv::Mat Irgb;
            cv::cvtColor(image, Irgb, cv::COLOR_BGR2RGB);

            Resizer resizer(Irgb, detector->getWindowSize(), minWidth);

            std::vector<drishti::face::FaceModel> faces;
            const auto& Hdr = resizer.getDetectorToRegressor();
            (*detector)(resizer.getPlanar(), resizer.getPadded(), faces, Hdr);

            resizer(faces);

            if (!doPositiveOnly || (faces.size() > 0))
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(frame.name);
                std::string filename = sOutput + "/" + base;

                logger->info("{}/{} {} = {}", ++total, video->count(), filename, faces.size());

                // Save detection results in JSON:
                if (!writeAsJson(filename + ".json", faces))
                {
                    logger->error("Failed to write: {}.json", filename);
                }

#if defined(DRISHTI_USE_IMSHOW)
                int windowCount = 0;
                drishti::core::scope_guard waiter = [&]() {
                    if (windowCount > 0)
                    {
                        glfw::waitKey(doPause ? 0 : 1);
                    }
                };
#endif

                if (doEyes)
                {
                    for (int i = 0; i < faces.size(); i++)
                    {
                        cv::Mat eyes = cropEyes(image, faces[i], { 640, 240 }, 0.666f, doAnnotation);

                        std::stringstream ss;
                        ss << std::setfill('0') << std::setw(2) << i;
                        cv::imwrite(filename + ss.str() + "_eyes.png", eyes);

#if defined(DRISHTI_USE_IMSHOW)
                        if (doDisplay)
                        {
                            windowCount++;
                            glfw::imshow("eyes", eyes);
                        }
#endif
                    }
                }

                if (doAnnotation)
                {
                    cv::Mat canvas = image.clone();
                    drawObjects(canvas, faces);
                    cv::imwrite(filename + "_faces.png", canvas);

#if defined(DRISHTI_USE_IMSHOW)
                    if (doDisplay)
                    {
                        windowCount++;
                        glfw::imshow("face", canvas);
                    }
#endif
                }
            }
        }
    };

    if (threads == 1 || threads == 0 || doDisplay || !video->isRandomAccess())
    {
        harness({ 0, static_cast<int>(video->count()) });
    }
    else
    {
        cv::parallel_for_({ 0, static_cast<int>(video->count()) }, harness, std::max(threads, -1));
    }

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

    return 0;
}

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
        logger->error("Specified model file does not exist or is not readable");
        return 1;
    }
    return 0;
}

static cv::Mat
cropEyes(const cv::Mat& image, const drishti::face::FaceModel& face, const cv::Size& size, float scale, bool annotate)
{
    const cv::Matx33f H = drishti::face::FaceStabilizer::stabilize(face, size, scale);

    cv::Mat eyes;
    cv::warpAffine(image, eyes, H.get_minor<2, 3>(0, 0), size);

    if (annotate)
    {
        const auto face2 = H * face;
        face2.eyeFullL->draw(eyes);
        face2.eyeFullR->draw(eyes);
    }

    return eyes;
}

static bool
writeAsJson(const std::string& filename, const std::vector<drishti::face::FaceModel>& faces)
{
    std::ofstream ofs(filename);
    if (ofs)
    {
        cereal::JSONOutputArchive oa(ofs);
        typedef decltype(oa) Archive; // needed by macro

#if 0
        oa << GENERIC_NVP("objects", faces);
#else
        std::cerr << "Must be implemented" << std::endl;
#endif
    }
    return ofs.good();
}

static void
drawObjects(cv::Mat& canvas, const std::vector<drishti::face::FaceModel>& faces)
{
    for (const auto& f : faces)
    {
        f.draw(canvas, 2, true, true);
    }
}

#if defined(DRISHTI_USE_IMSHOW)
static void initWindow(const std::string& name)
{
    // Hack/workaround needed for continuous preview in current imshow lib
    cv::Mat canvas(240, 320, CV_8UC3, cv::Scalar(0, 255, 0));
    cv::putText(canvas, "GLFW Fix", { canvas.cols / 4, canvas.rows / 2 }, CV_FONT_HERSHEY_PLAIN, 2.0, { 0, 0, 0 });
    glfw::imshow(name.c_str(), canvas);
    glfw::waitKey(1);
    glfw::destroyWindow(name.c_str());
}
#endif
