/*!
  @file   eye.cpp
  @author David Hirvonen
  @brief  Console app to fit eye models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/core/LazyParallelResource.h"
#include "drishti/core/Line.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/padding.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/testlib/drishti_cli.h"

// Package includes:
#include "thread_pool/thread_pool.hpp"
#include "cxxopts.hpp"
#include <opencv2/highgui.hpp> // cv::imwrite
#include <cereal/archives/json.hpp>

// System includes:
#include <condition_variable>

/*
 * Write an eye model in JSON format:
 */

static bool writeAsJson(const std::string &filename, drishti::eye::EyeModel &eye)
{
    std::ofstream ofs(filename);
    if(ofs)
    {
        cereal::JSONOutputArchive oa(ofs);
        typedef decltype(oa) Archive; // needed by macro
        oa << GENERIC_NVP("eye", eye);
    }
    return ofs.good();
}

// Use drishti_main to support cross platform interface:
int drishti_main(int argc, char **argv)
{
    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-eye");
    
    // ############################
    // ### Command line parsing ###
    // ############################
    
    std::string sInput, sOutput, sModel;
    bool doThreads = true, doJson = true, doAnnotation = false;
    
    cxxopts::Options options("drishti-eye", "Command line interface for eye model fitting");
    options.add_options()
    ("i,input", "Input file", cxxopts::value<std::string>(sInput))
    ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
    ("m,model", "Input file", cxxopts::value<std::string>(sModel))
    ("t,threads", "Multi-threaded", cxxopts::value<bool>(doThreads))
    ("j,json", "JSON model output", cxxopts::value<bool>(doJson))
    ("a,annotate", "Create annotated images", cxxopts::value<bool>(doAnnotation))
    ;
    
    options.parse(argc, argv);
    
    // ############################################
    // ### Command line argument error checking ###
    // ############################################
    
    // ### Output options:
    if(!(doJson || doAnnotation))
    {
        logger->error() << "Must specify one of: --json or --annotate";
        return 1;
    }
    
    // ### Directory
    if(sOutput.empty())
    {
        logger->error() << "Must specify output directory";
        return 1;
    }
    if(!drishti::cli::file::exists(sOutput + "/.drishti"))
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
    using EyeModelEstimatorPtr = std::unique_ptr<drishti::eye::EyeModelEstimator>;
    drishti::core::LazyParallelResource<std::thread::id, EyeModelEstimatorPtr> manager = [&]()
    {
        return drishti::core::make_unique<drishti::eye::EyeModelEstimator>(sModel);
    };
    
    // Shared state:
    std::mutex mutex;
    std::size_t total = 0;
    
    // Parallel loop:
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto &segmenter = manager[std::this_thread::get_id()];
        assert(segmenter);
        
        // Load current image
        cv::Mat image = cv::imread(filenames[i], cv::IMREAD_COLOR);
        if(!image.empty())
        {
            cv::Mat padded;
            auto tl = drishti::core::padToAspectRatio(image, padded, 4.0/3.0, false);
            
            drishti::eye::EyeModel eye;
            (*segmenter)(padded, eye);
            eye -= tl; // remove padding offset
            
            if(!sOutput.empty())
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(filenames[i]);
                std::string filename = sOutput + "/" + base;
                
                logger->info() << total++ << "/" << filenames.size() << " " << filename;

                // Write the annotated image
                if(doAnnotation)
                {
                    cv::Mat canvas = image.clone();
                    eye.draw(canvas);
                    cv::imwrite(filename + ".png", canvas);
                }

                // Save eye model results as xml:
                if(doJson)
                {
                    if(!writeAsJson(filename + ".json", eye))
                    {
                        logger->error() << "Failed to write: " << filename << ".json";
                    }
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
    }
}

