/*! -*-c++-*-
  @file   eye.cpp
  @author David Hirvonen
  @brief  Console app to fit eye models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Local includes:
#include "drishti/core/drishti_stdlib_string.h" // android workaround
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
#include "cxxopts.hpp"

#include <opencv2/imgproc.hpp> // warpPerspective
#include <opencv2/highgui.hpp> // cv::imwrite

#include <spdlog/fmt/ostr.h>

#include <cereal/archives/json.hpp>

// System includes:
#include <condition_variable>

DRISHTI_BEGIN_NAMESPACE(drishti)
DRISHTI_BEGIN_NAMESPACE(eye)

/*
 * Write an eye model in JSON format:
 */

static bool writeAsJson(const std::string& filename, drishti::eye::EyeModel& eye)
{
    std::ofstream ofs(filename);
    if (ofs)
    {
        cereal::JSONOutputArchive oa(ofs);
        typedef decltype(oa) Archive; // needed by macro
        oa << GENERIC_NVP("eye", eye);
    }
    return ofs.good();
}

/*
 * Invertible/cascadable preprocessing transformation class (eye specific):
 * 1) apply image transformation in constructor.
 * 2) undo transformation to estimated model in destructor.
 */

struct Transform
{
    Transform(drishti::eye::EyeModel& eye)
        : eye(eye)
    {
    }
    operator cv::Mat() { return image; }
    cv::Mat image;               // transformed image
    drishti::eye::EyeModel& eye; // handle image to transform
};

struct Padder : public Transform
{
    Padder(const cv::Mat& input, drishti::eye::EyeModel& eye, float aspectRatio = 4.f / 3.f)
        : Transform(eye)
    {
        if (!isGoodAspectRatio(input.size(), aspectRatio))
        {
            tl = drishti::core::padToAspectRatio(input, image, aspectRatio, false);
        }
        else
        {
            image = input;
        }
    }
    ~Padder()
    {
        eye = eye - tl;
    }

    static bool isGoodAspectRatio(const cv::Size& size, float targetAspectRatio)
    {
        const float currentAspectRatio = std::abs(static_cast<float>(size.width) / static_cast<float>(size.height));
        return (std::abs(targetAspectRatio - currentAspectRatio) < 0.1f);
    }

    cv::Point tl;
};

struct Flopper : public Transform
{
    Flopper(const cv::Mat& input, drishti::eye::EyeModel& eye, bool isRight)
        : Transform(eye)
        , isRight(isRight)
    {
        if (!isRight)
        {
            cv::flip(input, image, 1);
        }
        else
        {
            image = input;
        }
    }

    ~Flopper()
    {
        if (!isRight)
        {
            eye.flop(image.cols);
        }
    }

    bool isRight = true;
};

struct Warper : public Transform
{
    Warper(const cv::Mat& input, drishti::eye::EyeModel& eye, const cv::Matx33f* H)
        : Transform(eye)
        , H(H)
    {
        if (H)
        {
            cv::warpPerspective(input, image, cv::Mat1f(*H), input.size(), cv::INTER_LINEAR);
        }
        else
        {
            image = input;
        }
    }

    ~Warper()
    {
        if (H)
        {
            eye = H->inv() * eye;
        }
    }

    const cv::Matx33f* H = nullptr;
};

static void fitEyeModel(eye::EyeModelEstimator& fitter, cv::Mat& image, eye::EyeModel& eye, bool isRight, const cv::Matx33f* prewarp = nullptr)
{
    Warper warper(image, eye, prewarp);
    { // First scope provides padding transformation
        static const float targetAspectRatio = 4.0 / 3.0;
        Flopper flopper(image, eye, isRight);
        { // Second scope provides LEFT vs RIGHT:
            Padder padder(flopper, eye, targetAspectRatio);
            fitter(padder, eye);
        }
    }
}

DRISHTI_END_NAMESPACE(eye)
DRISHTI_END_NAMESPACE(drishti)

// Use gauze_main to support cross platform interface:
int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-eye");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput, sModel, sPrewarp;
    int threads = -1;
    int stages = std::numeric_limits<int>::max();
    bool doJson = true;
    bool doAnnotation = false;
    bool isRight = false;
    bool isLeft = false;
    bool doLabels = false;

    cv::Matx33f prewarp = cv::Matx33f::eye();

    cxxopts::Options options("drishti-eye", "Command line interface for eye model fitting");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("m,model", "Eye model file", cxxopts::value<std::string>(sModel))
        ("s,stages", "Restrict cascade to <s> stages", cxxopts::value<int>(stages))
        ("t,threads", "Thread count", cxxopts::value<int>(threads))
        ("j,json", "JSON model output", cxxopts::value<bool>(doJson))
        ("a,annotate", "Create annotated images", cxxopts::value<bool>(doAnnotation))
        ("r,right", "Right eye inputs", cxxopts::value<bool>(isRight))
        ("l,left", "Left eye inputs", cxxopts::value<bool>(isLeft))
        ("p,prewarp", "Prewarp", cxxopts::value<std::string>(sPrewarp))
        ("L,labels", "Generate label image", cxxopts::value<bool>(doLabels))
        ("h,help", "Print help message");
    // clang-format on    
    
    options.parse(argc, argv);
    
    if((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({""}) << std::endl;
        return 0;
    }
    
    // ############################################
    // ### Command line argument error checking ###
    // ############################################

    // Be pedantic about input orientation to ensure correct results:
    if((options["right"].count() + options["left"].count()) != 1)
    {
        logger->error("Must specify -right or -left but not both!");
        return 1;
    }
    
    isRight = !isLeft;
    
    // ### Output options:
    if(!(doJson || doAnnotation))
    {
        logger->error("Must specify one of: --json or --annotate");
        return 1;
    }
    
    // ### Directory
    if(sOutput.empty())
    {
        logger->error("Must specify output directory");
        return 1;
    }
    
    if(drishti::cli::directory::exists(sOutput, ".drishti-eye"))
    {
        std::string filename = sOutput + "/.drishti-eye";
        remove(filename.c_str());
    }
    else
    {
        logger->error("Specified directory {} does not exist or is not writeable", sOutput);
        return 1;
    }
    
    // ### Model
    if(sModel.empty())
    {
        logger->error("Must specify model file");
        return 1;
    }
    if(!drishti::cli::file::exists(sModel))
    {
        logger->error("Specified model file does not exist or is not readable");
        return 1;
    }
    
    // ### Input
    if(sInput.empty())
    {
        logger->error("Must specify input image or list of images");
        return 1;
    }
    if(!drishti::cli::file::exists(sInput))
    {
        logger->error("Specified input file does not exist or is not readable");
        return 1;
    }
    
    // ### prewarp
    bool hasPrewarp = false;
    if(!sPrewarp.empty())
    {
        std::vector<std::string> tokens;
        drishti::core::tokenize(sPrewarp, tokens);
        if(tokens.size() == 9)
        {
            for(int i = 0, y = 0; y < 3; y++)
            {
                for(int x = 0; x < 3; x++, i++)
                {
                    prewarp(y,x) = std::stof(tokens[i]);
                }
            }
            hasPrewarp = true;
        }
    }
    
    if(hasPrewarp)
    {
        logger->info("prewarp: {}", prewarp);
    }

    const auto filenames = drishti::cli::expand(sInput);
    
    // Allocate resource manager:
    using EyeModelEstimatorPtr = std::unique_ptr<drishti::eye::EyeModelEstimator>;
    drishti::core::LazyParallelResource<std::thread::id, EyeModelEstimatorPtr> manager = [&]()
    {
        return drishti::core::make_unique<drishti::eye::EyeModelEstimator>(sModel);
    };
    
    std::size_t total = 0;
    
    // Parallel loop:
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto &segmenter = manager[std::this_thread::get_id()];
        segmenter->setEyelidStagesHint(stages);
        assert(segmenter);
        
        // Load current image
        cv::Mat image = cv::imread(filenames[i], cv::IMREAD_COLOR);
        if(!image.empty())
        {
            drishti::eye::EyeModel eye;
            drishti::eye::fitEyeModel(*segmenter, image, eye, isRight, hasPrewarp ? &prewarp : nullptr);
            eye.refine();

            if(!sOutput.empty())
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(filenames[i]);
                std::string filename = sOutput + "/" + base;
                
                logger->info("{}/{} {}", ++total, filenames.size(), filename);

                // Write the annotated image
                if(doAnnotation)
                {
                    cv::Mat canvas = image.clone();
                    eye.draw(canvas);
                    cv::imwrite(filename + "_contours.png", canvas);
                }

                // Save part labels
                if(doLabels)
                {
                    cv::Mat labels = eye.labels(image.size());
                    cv::imwrite(filename + "_labels.png", labels);
                }

                // Save eye model results as xml:
                if(doJson)
                {
                    if(!drishti::eye::writeAsJson(filename + ".json", eye))
                    {
                        logger->error("Failed to write: {}.json", filename);
                    }
                }
            }
        }
    };
    
    if(threads == 1 || threads == 0)
    {
        harness({0,static_cast<int>(filenames.size())});
    }
    else
    {
        cv::parallel_for_({0,static_cast<int>(filenames.size())}, harness, std::max(threads, -1));
    }

    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch(std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }
    
    return 0;
}

