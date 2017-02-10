/*!
  @file   face.cpp
  @author David Hirvonen
  @brief  Face detection and landmark estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/acf/ACF.h"
#include "drishti/face/FaceDetector.h"
#include "drishti/core/LazyParallelResource.h"
#include "drishti/core/Line.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/testlib/drishti_cli.h"

// Package includes:
#include "cxxopts.hpp"
#include <opencv2/highgui.hpp>
#include <cereal/archives/json.hpp>

static bool writeAsJson(const std::string &filename, const std::vector<drishti::face::FaceModel> &faces);
static void drawObjects(cv::Mat &canvas, const std::vector<drishti::face::FaceModel> &faces);

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
    Resizer(cv::Mat &image, const cv::Size &winSize, int width = -1)
    {
        if((width >= 0) && !image.empty())
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
            padded = PaddedImage(green, {{0,0},green.size()});
        }

        { // Create the lower resolution image for detection:
            cv::Mat It = reduced.t(), Itf;
            It.convertTo(Itf, CV_32FC3, 1.0f/255.f);
            planar = MatP(Itf);
            Sdr = 1.f/Sfd;
        }        
        
        // Store the transformation from the detection to the regression image:
        Hdr = cv::Matx33f(cv::Matx33f::diag({Sdr,Sdr,1.f}));

        // Compute and store the transformation from the regressor to full resolution:
        const float Srd = (1.f/Sdr), Sdf = (1.f/Sfd);
        Srf = Sdf * Srd;
        Hrf = cv::Matx33f::diag({Srf,Srf,1.f}); // regressor->full
    }

    // Faces will be avaialble at the regressor resolution:
    void operator()(std::vector<drishti::face::FaceModel> &faces) const
    {
        if(Srf != 1.f)
        {
            for(auto &f : faces)
            {
                f = Hrf * f;
            }
        }
    }

    const cv::Matx33f &getRegressorToFull() const { return Hrf; }    
    const cv::Matx33f &getDetectorToRegressor() const { return Hdr; }
    
    const MatP &getPlanar() const { return planar; }
    MatP &getPlanar() { return planar; }
    
    const PaddedImage &getPadded() const { return padded; }
    PaddedImage &getPadded() { return padded; }

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

using LoggerPtr = std::shared_ptr<spdlog::logger>;
static bool checkModel(LoggerPtr &logger, const std::string &sModel, const std::string &description)
{
    if(sModel.empty())
    {
        logger->error() << "Must specify valid model " << sModel;
        return 1;
    }
    if(!drishti::cli::file::exists(sModel))
    {
        logger->error() << "Specified model file does not exist or is not readable";
        return 1;
    }
    return 0;
}
    
int drishti_main(int argc, char **argv)
{
    const auto argumentCount = argc;
    
    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-acf");
    
    // ############################
    // ### Command line parsing ###
    // ############################
    
    std::string sInput, sOutput, sModel;
    bool doThreads = true;
    bool doAnnotation = false;
    bool doPositiveOnly = false;
    double cascCal = 0.0;
    int minWidth = -1; // minimum object width
    int maxWidth = -1; // maximum object width TODO

    // Full set of models must be specified:
    std::string sFaceDetector;
    std::string sFaceDetectorMean;
    std::string sFaceRegressor;
    std::string sEyeRegressor;
    
    cxxopts::Options options("drishti-acf", "Command line interface for ACF object detection (see Piotr's toolbox)");
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
    
        // Detection parameters:
        ("l,min", "Minimum object width (lower bound)", cxxopts::value<int>(minWidth))
        ("c,calibration", "Cascade calibration", cxxopts::value<double>(cascCal))

        // Clasifier and regressor models:
        ("D,detector", "Face detector model", cxxopts::value<std::string>(sFaceDetector))
        ("M,mean", "Face detector mean", cxxopts::value<std::string>(sFaceDetectorMean))
        ("R,regressor", "Face regressor", cxxopts::value<std::string>(sFaceRegressor))
        ("E,eye", "Eye model", cxxopts::value<std::string>(sEyeRegressor))
    
        // Output parameters:
        ("a,annotate", "Create annotated images", cxxopts::value<bool>(doAnnotation))
        ("p,positive", "Limit output to positve examples", cxxopts::value<bool>(doPositiveOnly))
        ("t,threads", "Multi-threaded", cxxopts::value<bool>(doThreads))
        ("h,help", "Print help message");
    
    options.parse(argc, argv);
    
    if((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({""}) << std::endl;
        return 0;
    }
    
    // ############################################
    // ### Command line argument error checking ###
    // ############################################

    // ### Directory
    if(sOutput.empty())
    {
        logger->error() << "Must specify output directory";
        return 1;
    }
    
    if(drishti::cli::directory::exists(sOutput, ".drishti-acf"))
    {
        std::string filename = sOutput + "/.drishti-acf";
        remove(filename.c_str());
    }
    else
    {
        logger->error() << "Specified directory " << sOutput << " does not exist or is not writeable";
        return 1;
    }
    
    // ### Input
    if(sInput.empty())
    {
        logger->error() << "Must specify input image or list of images";
        return 1;
    }
    if(!drishti::cli::file::exists(sInput))
    {
        logger->error() << "Specified input file does not exist or is not readable";
        return 1;
    }

    // Check for valid models
    std::vector<std::pair<std::string, std::string>> config
    {
        { sFaceDetector, "face-detector"},
        { sFaceDetector, "face-detector-mean"},
        { sFaceDetector, "face-regressor"},
        { sFaceDetector, "eye-regressor"}
    };
    
    for(const auto &c : config)
    {
        if(checkModel(logger, c.first, c.second))
        {
            return 1;
        }
    }
    
    const auto filenames = drishti::cli::expand(sInput);
    
    // Allocate resource manager:
    using FaceDetectorPtr = std::unique_ptr<drishti::face::FaceDetector>;
    drishti::core::LazyParallelResource<std::thread::id, FaceDetectorPtr> manager = [&]()
    {        
        auto factory = std::make_shared<drishti::face::FaceDetectorFactory>();
        factory->sFaceDetector = sFaceDetector;
        factory->sFaceRegressors = { sFaceRegressor };
        factory->sEyeRegressor = sEyeRegressor; 
        factory->sFaceDetectorMean = sFaceDetectorMean;

        FaceDetectorPtr detector = drishti::core::make_unique<drishti::face::FaceDetector>(*factory);

        if(detector)
        {
            // Cofigure parameters:
            detector->setDoNMS(true);
            
            auto acf = dynamic_cast<drishti::acf::Detector*>(detector->getDetector());
            if(acf && acf->good())
            {
                // Cascade threhsold adjustment:
                if(cascCal != 0.f)
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
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto &detector = manager[std::this_thread::get_id()];
        assert(detector);

        // Load current image
        cv::Mat image = cv::imread(filenames[i], cv::IMREAD_COLOR);
        
        if(!image.empty())
        {
            cv::Mat Irgb;
            cv::cvtColor(image, Irgb, cv::COLOR_BGR2RGB);
        
            Resizer resizer(Irgb, detector->getWindowSize(), minWidth);

            std::vector<drishti::face::FaceModel> faces;
            const auto &Hdr = resizer.getDetectorToRegressor();
            (*detector)(resizer.getPlanar(), resizer.getPadded(), faces, Hdr);
            
            resizer(faces);

            if(!doPositiveOnly || (faces.size() > 0))
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(filenames[i]);
                std::string filename = sOutput + "/" + base;
                
                logger->info() << ++total << "/" << filenames.size() << " " << filename << " = " << faces.size();

                // Save detection results in JSON:                
                if(!writeAsJson(filename + ".json", faces))
                {
                    logger->error() << "Failed to write: " << filename << ".json";
                }

                if(doAnnotation)
                {
                    cv::Mat canvas = image.clone();
                    drawObjects(canvas, faces);
                    cv::imwrite(filename + "_faces.png", canvas);
                }
            }
        }
    };
    
    if(doThreads)
    {
        cv::parallel_for_({0,static_cast<int>(filenames.size())}, harness);
    }
    else
    {
        harness({0,static_cast<int>(filenames.size())});
    }

    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return drishti_main(argc, argv);
    }
    catch(std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    catch(...)
    {
        std::cerr << "Unknown exception";
    }
    
    return 0;
}

// utility

static bool
writeAsJson(const std::string &filename, const std::vector<drishti::face::FaceModel> &faces)
{
    std::ofstream ofs(filename);
    if(ofs)
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
drawObjects(cv::Mat &canvas, const std::vector<drishti::face::FaceModel> &faces)
{
    for(const auto &f : faces)
    {
        f.draw(canvas, 2, true);
    }
}

