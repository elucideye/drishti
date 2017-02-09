/*!
  @file   acf.cpp
  @author David Hirvonen
  @brief  Aggregated channel feature computation and object detection.

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

static bool writeAsJson(const std::string &filename, const std::vector<cv::Rect> &objects);
static void drawObjects(cv::Mat &canvas, const std::vector<cv::Rect> &objects);
static cv::Rect2f operator*(const cv::Rect2f &roi, float scale);

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
    Resizer(cv::Mat &image, const cv::Size &winSize, int width = -1)
    {
        if((width >= 0) && !image.empty())
        {
            scale = static_cast<float>(winSize.width) / static_cast<float>(width);
            const int interpolation = (scale < 1.f) ? cv::INTER_AREA : cv::INTER_LINEAR;
            cv::resize(image, reduced, {}, scale, scale, interpolation);
        }
        else
        {
            reduced = image;
        }
    }

    void operator()(std::vector<cv::Rect> &objects) const
    {
        if(scale != 1.f)
        {
            for(auto &o : objects)
            {
                o = cv::Rect2f(o) * (1.f/scale);
            }
        }
    }
    operator cv::Mat() { return reduced; }
    
    float scale = 1.f;
    cv::Mat reduced;
};

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
    bool doNms = false;
    double cascCal = 0.0;
    int minWidth = -1; // minimum object width
    int maxWidth = -1; // maximum object width TODO
    
    cxxopts::Options options("drishti-acf", "Command line interface for ACF object detection (see Piotr's toolbox)");
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("m,model", "Input file", cxxopts::value<std::string>(sModel))
        ("n,nms", "Non maximum suppression", cxxopts::value<bool>(doNms))
        ("l,min", "Minimum object width (lower bound)", cxxopts::value<int>(minWidth))
        ("c,calibration", "Cascade calibration", cxxopts::value<double>(cascCal))
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
    
    // ### Model
    if(sModel.empty())
    {
        logger->error() << "Must specify model file";
        return 1;
    }
    if(!drishti::cli::file::exists(sModel))
    {
        logger->error() << "Specified model file does not exist or is not readable";
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

    const auto filenames = drishti::cli::expand(sInput);
    
    // Allocate resource manager:
    using AcfPtr = std::unique_ptr<drishti::acf::Detector>;
    drishti::core::LazyParallelResource<std::thread::id, AcfPtr> manager = [&]()
    {
        AcfPtr acf = drishti::core::make_unique<drishti::acf::Detector>(sModel);
        if(acf.get() && acf->good())
        {
            // Cofigure parameters:
            acf->setDoNonMaximaSuppression(doNms);
            
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
            acf.release();
        }
        
        return acf;
    };
    
    std::size_t total = 0;
    
    // Parallel loop:
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto &detector = manager[std::this_thread::get_id()];
        assert(detector);

        // Load current image
        cv::Mat image = cv::imread(filenames[i], cv::IMREAD_COLOR), imageRGB;
        cv::cvtColor(image, imageRGB, cv::COLOR_BGR2RGB);
        
        if(!image.empty())
        {
            Resizer resizer(imageRGB, detector->getWindowSize(), minWidth);

            std::vector<double> scores;
            std::vector<cv::Rect> objects;
            (*detector)(resizer, objects, &scores);
            
            resizer(objects);

            if(!doPositiveOnly || (objects.size() > 0))
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(filenames[i]);
                std::string filename = sOutput + "/" + base;
                
                logger->info() << ++total << "/" << filenames.size() << " " << filename << " = " << objects.size();

                // Save detection results in JSON:                
                if(!writeAsJson(filename + ".json", objects))
                {
                    logger->error() << "Failed to write: " << filename << ".json";
                }

                if(doAnnotation)
                {
                    cv::Mat canvas = image.clone();
                    drawObjects(canvas, objects);
                    cv::imwrite(filename + "_objects.png", canvas);
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

static bool writeAsJson(const std::string &filename, const std::vector<cv::Rect> &objects)
{
    std::ofstream ofs(filename);
    if(ofs)
    {
        cereal::JSONOutputArchive oa(ofs);
        typedef decltype(oa) Archive; // needed by macro
        oa << GENERIC_NVP("objects", objects);
    }
    return ofs.good();
}

static void drawObjects(cv::Mat &canvas, const std::vector<cv::Rect> &objects)
{
    for(const auto &o : objects)
    {
        cv::rectangle(canvas, o, {0,255,0}, 2, 8);
    }
}

static cv::Rect2f operator*(const cv::Rect2f &roi, float scale)
{
    return { roi.x * scale, roi.y * scale, roi.width * scale, roi.height * scale };
}

