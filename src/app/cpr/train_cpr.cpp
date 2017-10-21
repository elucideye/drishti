/*! -*-c++-*-
 @file   train_cpr.cpp
 @author David Hirvonen
 @brief  Train casecaded pose regression ellipse model.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 Dimensions: { xs, ys, ang, scl, asp }
 
 Note: The intial default global cascade + regression parameters
 
   // i.e., such as these
   cprPrm.ftrPrm->type = { "type", 2 };
   cprPrm.ftrPrm->F = { "F", F };
   cprPrm.ftrPrm->radius = { "radius", 1.66 };
 
 have been replaced with per stage recipes that allow a more
 tailored regression policy.
 
 */

//#define BOOST_SPIRIT_DEBUG

#include "drishti/eye/Eye.h"
#include "drishti/rcpr/CPR.h"
#include "drishti/rcpr/CPRIO.h"
#include "drishti/core/Line.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_serialize.h"
#include "drishti/core/Parallel.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/testlib/drishti_cli.h"

#include "RecipeIO.h"
#include "EyeIO.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/imgproc/imgproc_c.h>
#include <opencv2/imgcodecs/imgcodecs.hpp>
#include <opencv2/highgui/highgui.hpp>

#include "cxxopts.hpp"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include <boost/filesystem.hpp>
#include <boost/fusion/adapted/std_pair.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/repository/include/qi_seek.hpp>

#define DO_SANITY_CHECK 0
#define TRAIN_CPR_DEBUG_LOAD 0
#define TRAIN_CPR_MIN_IRIS_DIAMETER 10.0

namespace bfs = boost::filesystem;

using drishti::rcpr::Vector1d;

struct PoseData
{
    drishti::rcpr::ImageMaskPairVec images;
    drishti::rcpr::EllipseVec ellipses[2];
    std::vector<cv::Matx33f> H; // normalization
};

using drishti::geometry::operator*;
using StringVec = std::vector<std::string>;

struct EllipseSamples
{
    EllipseSamples(std::shared_ptr<spdlog::logger>& logger);
    void load(const std::string& filename, int width, const std::string& sExt, bool doIris);

    std::shared_ptr<spdlog::logger> logger;
    PoseData samples;
    drishti::rcpr::Vector1d mu;
    drishti::rcpr::Vector1d sigma;
};

#if defined(DRISHTI_USE_IMSHOW) && TRAIN_CPR_DEBUG_LOAD
static cv::Mat previewSample(const cv::Mat& image, drishti::rcpr::CPR::CPRResult& result, drishti::rcpr::Vector1d& mu)
{
    cv::Mat canvas;
    cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);
    cv::ellipse(canvas, drishti::rcpr::phiToEllipse(result.p), { 0, 255, 0 }, 1, 8);
    cv::ellipse(canvas, drishti::rcpr::phiToEllipse(mu), { 255, 0, 0 }, 1, 8);
    return canvas;
}
#endif

DRISHTI_BEGIN_NAMESPACE(cpr)
static void load(const std::string& sEye, drishti::eye::EyeModel& eye);
static int writeDefaultRecipe(const std::string& filename, const std::string& dimensions);
static std::vector<float> parseVec5f(const std::string& dimensions);
void ellipse(cv::Mat canvas, const cv::RotatedRect& e, const cv::Scalar& color = cv::Scalar::all(255), int width = 1, int type = 8);
DRISHTI_END_NAMESPACE(cpr)

int gauze_main(int argc, char** argv)
{
    auto logger = drishti::core::Logger::create("train_cpr");

    const auto argumentCount = argc;

    std::string sWeights = "{1.0,2.0,8.0,32.0,64.0,1.0}";
    std::string sDimensions = "{{0,1},{3},{0,1},{3},{0,1},{3},{0,1},{3}}";
    std::string sExtension = ".eye.xml";
    std::string sTrain;
    std::string sTest;
    std::string sModel;
    std::string sRecipe;
    std::string sTemplate;
    std::string sTestLog;
    std::string sLoggingDir;

    bool doWindow = false;

    int targetWidth = 128;
    int T = 0; // # of stages
    int L = 4; // oversampling
    int F = 200;

    // TODO: support generic ellipse regression (no iris assumption)
    const bool doIris = true;

    bool doSilent = false;
    bool doVerbose = false;
    bool doHelp = false;
    bool doPupil = false;

    cxxopts::Options options("train_shape_predictor", "Command line interface for dlib shape_predictor training");

    // clang-format off
    options.add_options()
        ( "dimensions", "List of dimensions in cascaded pose regression", cxxopts::value<std::string>(sDimensions) )
        ( "weights", "Per element weights for loss: {xs,ys,ang,scl,asp}", cxxopts::value<std::string>(sWeights) )
        ( "extension", "Filename extension", cxxopts::value<std::string>(sExtension) )
        ( "train", "Train samples", cxxopts::value<std::string>(sTrain) )
        ( "test", "Test samples", cxxopts::value<std::string>(sTest))
        ( "model", "Model filename", cxxopts::value<std::string>(sModel) )
        ( "test-log", "Testing log", cxxopts::value<std::string>(sTestLog) )
        ( "recipe", "Training recipe", cxxopts::value<std::string>(sRecipe) )
        ( "template", "Training recipe template", cxxopts::value<std::string>(sTemplate) )
        ( "verbose", "Print verbose diagnostics", cxxopts::value<bool>(doVerbose) )
        ( "silent", "Disable logging entirely.", cxxopts::value<bool>(doSilent) )
        ( "pupil", "Train on pupil (else iris)", cxxopts::value<bool>(doPupil))
    
#if defined(DRISHTI_USE_IMSHOW)        
        ( "window", "Do window", cxxopts::value<bool>(doWindow) )
#endif
        ( "log", "Image logging directory", cxxopts::value<std::string>(sLoggingDir) )
        
        ( "help", "Print the help message", cxxopts::value<bool>(doHelp) );
    // clang-format on

    options.parse(argc, argv);

    if (doSilent)
    {
        logger->set_level(spdlog::level::off); // by default...
    }

    if ((argumentCount <= 1) || options.count("help"))
    {
        logger->info(options.help({ "" }));
        return 0;
    }

    // i.e., { xs, ys, ang, scl, asp }
    if (!sTemplate.empty())
    {
        return cpr::writeDefaultRecipe(sTemplate, sDimensions);
    }

    if (sTrain.empty())
    {
        logger->error("Must specify valid images.");
        return 1;
    }

    if (sRecipe.empty())
    {
        logger->error("Must specify valid training recipe.");
        return 1;
    }

    // ### Directory
    if (!sLoggingDir.empty())
    {
        if (drishti::cli::directory::exists(sLoggingDir, ".drishti-cpr"))
        {
            std::string filename = sLoggingDir + "/.drishti-cpr";
            remove(filename.c_str());
        }
        else
        {
            logger->error("Specified directory {} is not writeable", sLoggingDir);
            return 1;
        }
    }

    { // Test that we can write to the specified output model before starting training
        std::ofstream check(sModel);
        if (check)
        {
            remove(sModel.c_str());
        }
        else
        {
            logger->error("Cannot create specified output model {}", sModel);
            return 1;
        }
    }

    EllipseSamples train(logger);
    train.load(sTrain, targetWidth, sExtension, doIris);

    drishti::rcpr::CPR::Model model;
    drishti::rcpr::createModel(0, model);

    // Defaults:
    // ftrPrm = struct('type',2,'F',F,'radius',1.66);
    // fernPrm = struct('thrr',[-1 1]/5,'reg',.01,'S',5,'M',1,'R',R,'eta',1);
    // cprPrm = struct('model',model,'T',T,'L',L,'ftrPrm',ftrPrm,'fernPrm',fernPrm,'regModel',[],'verbose',1 );

    drishti::rcpr::CPR::CprPrm cprPrm;

    cpr::loadJSON(sRecipe, cprPrm.cascadeRecipes);
    T = cprPrm.cascadeRecipes.size();

    cprPrm.L = L;
    cprPrm.T = T;
    cprPrm.model = model;

    // Note: feature parameters can be overriden
    cprPrm.ftrPrm->type = 2;
    cprPrm.ftrPrm->F = F;
    cprPrm.ftrPrm->radius = 1.66;

    // Per-dimmension coefficients for weighted ellipse loss function.
    // The original CPR approach trained regressors for each coordinate
    // and selected the one that minized the global loss at that stage.
    // In such a min() configuration the loss function plays a more
    // critical role.  When using the recommended automatic weights
    // for iris data, it would rarely choose to adjust some parameters
    // such as aspect ratio.
    //
    // The current approach employs a fixed "recipe" w/ index list
    // at each stage, which works well (predictably) in practice.
    //
    // Reference implementation: { "wts",  {0.313, 0.342, 11.339, 13.059, 48.0 } };

    Vector1d wts{ 1.0f, 1.0f, 8.0f, 32.0f, 64.0f };
    if (!sWeights.empty())
    {
        wts = cpr::parseVec5f(sWeights);
    }

    // Suggested weights
    // Note: hand tuned weights seem to work better in practice, as this
    // for(int i = 0; i < 5; i++)
    // {
    //     wts[i] = (1.0 / cv::pow(trainSigma[i], 2.0));
    // }

    cprPrm.model->parts->wts = wts;

    { // Train the model:
        drishti::rcpr::CPR cpr;
        cpr.setStreamLogger(logger);

        if (doWindow || !sLoggingDir.empty())
        {
            std::map<std::string, int> counter;

            // clang-off
            drishti::rcpr::CPR::ViewFunc viewer = [&](const std::string& name, const cv::Mat& image) {
                if (counter.find(name) == counter.end())
                {
                    counter[name] = 0;
                }
                else
                {
                    counter[name]++;
                }

#if defined(DRISHTI_USE_IMSHOW)
                if (doWindow)
                {
                    glfw::imshow(name.c_str(), image);
                    glfw::waitKey(1);
                }
#endif
                if (!sLoggingDir.empty())
                {
                    std::stringstream ss;
                    ss << sLoggingDir << "/" << name << "_" << std::setfill('0') << std::setw(4) << counter[name] << ".png";
                    cv::imwrite(ss.str(), image);
                }
            };
            // clang-on
            cpr.setViewer(viewer);
        }
        cpr.cprTrain(train.samples.images, train.samples.ellipses[int(doPupil)], train.samples.H, cprPrm, true);

        // Dump the model:
        save_cpb(sModel, cpr);
    }

    // Test the model
    if (!sTest.empty())
    {
        drishti::rcpr::CPR cpr;
        load_cpb(sModel, cpr);
        cpr.setStreamLogger(logger);

        EllipseSamples test(logger);
        test.load(sTest, targetWidth, sExtension, doIris);

        std::vector<double> errors(T, 0.0); // accumulate per stage errors
        for (int i = 0; i < test.samples.images.size(); i++)
        {
            drishti::rcpr::CPR::CPRResult result;
            cpr.cprApplyTree(test.samples.images[i], *cpr.regModel, *(cpr.regModel->pStar), result);

            // Measure error at each stage:
            for (int t = 0; t < T; t++)
            {
                errors[t] += drishti::rcpr::dist(*cprPrm.model, result.pAll[t], test.samples.ellipses[0][i]);
            }

#if defined(DRISHTI_USE_IMSHOW) && TRAIN_CPR_DEBUG_LOAD
            {
                cv::Mat canvas = previewSample(test.samples.images[i].getImage(), result, *(cpr.regModel->pStar));
                glfw::destroyWindow("I");
                glfw::imshow("I", canvas);
                glfw::waitKey(0);
            }
#endif
        }
        for (auto& e : errors)
        {
            e /= double(test.samples.images.size());
        }

        if (!sTestLog.empty())
        {
            std::ofstream os(sTestLog);
            if (os)
            {
                for (int t = 0; t < T; t++)
                {
                    os << errors[t] << std::endl;
                }
            }
        }
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
        std::cerr << e.what() << std::endl;
    }
}

EllipseSamples::EllipseSamples(std::shared_ptr<spdlog::logger>& logger)
    : logger(logger)
{
}

void EllipseSamples::load(const std::string& filename, int targetWidth, const std::string& sExt, bool doIris)
{
    const auto filenames = drishti::cli::expand(filename);

    cv::Mat features(filenames.size(), 5, CV_64F);

    samples.images.resize(filenames.size());
    samples.ellipses[0].resize(filenames.size());
    samples.ellipses[1].resize(filenames.size());
    samples.H.resize(filenames.size());

    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        // Image filename:
        const auto& sImage = filenames[i];

        cv::Mat I = cv::imread(sImage), canvas = I.clone();
        if (!I.empty())
        {
            if (I.channels() >= 3)
            {
                cv::extractChannel(I, I, 2); // use red channel
            }

            drishti::rcpr::Vector1d ellipse1, ellipse2;
            drishti::eye::EyeModel eye;

            // Eye filename:
            bfs::path sEye(sImage);
            sEye.replace_extension(sExt);
            cpr::load(sEye.string(), eye);

            // Capture iris before correction:
            const auto initialIris = eye.irisEllipse;

            // Make ellipse orientation "down" to correct rotational
            // ambiguities for circles.
            eye.normalize();
            eye.refine();

            const auto& e1 = eye.irisEllipse;
            const auto& e2 = eye.pupilEllipse;

            const float angularError = std::abs(e1.angle - 90.0);
            if (angularError > 45.0)
            {
                logger->error("Unexpected orientation {} for image {}", e1.angle, sImage);
            }

            // Need to transpose the model
            // Note: use eye corners instead of eyelids
            std::vector<cv::Point2f> eyelids{ eye.getInnerCorner(), eye.getOuterCorner() };
            samples.H[i] = drishti::geometry::procrustes(eyelids);

            // Ignore samples that are too small (or accidentally empty):
            if (doIris && (std::max(e1.size.width, e1.size.height) < TRAIN_CPR_MIN_IRIS_DIAMETER))
            {
                return;
            }

            CV_Assert(e1.angle >= 0.f);
            CV_Assert(e1.size.width > 0.f);
            CV_Assert(e2.size.width > 0.f);

            cv::Mat M = eye.mask(I.size(), false);

            const float scale = float(targetWidth) / I.cols;
            const cv::Matx33f S(cv::Matx33f::diag({ scale, scale, 1.f }));

            ellipse1 = drishti::rcpr::ellipseToPhi(S * e1);
            ellipse2 = drishti::rcpr::ellipseToPhi(S * e2);

            for (int j = 0; j < 5; j++)
            {
                features.at<double>(i, j) = ellipse1[j];
            }

            cv::resize(I, I, { targetWidth, I.rows * targetWidth / I.cols }, 0.0, 0.0, cv::INTER_CUBIC);
            cv::resize(M, M, { targetWidth, M.rows * targetWidth / M.cols }, 0.0, 0.0, cv::INTER_NEAREST);

            samples.H[i] = samples.H[i] * S.inv();
            samples.images[i] = { I, M };
            samples.ellipses[0][i] = ellipse1;
            samples.ellipses[1][i] = ellipse2;
        }
    };

    cv::parallel_for_({ 0, static_cast<int>(filenames.size()) }, harness);

    sigma = mu = { 0.f, 0.f, 0.f, 0.f, 0.f };

    for (int i = 0; i < 5; i++)
    {
        struct
        {
            cv::Scalar mu, sigma;
        } stats;
        cv::meanStdDev(features.col(i), stats.mu, stats.sigma);
        mu[i] = stats.mu[0];
        sigma[i] = stats.sigma[0];
    }
}

DRISHTI_BEGIN_NAMESPACE(cpr)

// Draw an oriented ellipse (show major axis)
void ellipse(cv::Mat canvas, const cv::RotatedRect& e, const cv::Scalar& color, int width, int type)
{
    drishti::geometry::Ellipse e_(e);
    cv::ellipse(canvas, e, color, width, type);
    cv::line(canvas, e_.center, e_.getMajorAxisPos(), color, width, type);
}

// ######################################################
// ### BNF parser:                                    ###
// ### vector<vector<int>> = {{0,1}, {3}, {0,1}, {2}} ###
// ######################################################

template <typename Iterator, typename Skipper = qi::blank_type>
struct list_list_int_parser : qi::grammar<Iterator, std::vector<std::vector<int>>(), Skipper>
{
    list_list_int_parser()
        : list_list_int_parser::base_type(start)
    {
        // http://stackoverflow.com/a/40876962
        integer %= qi::int_[qi::_pass = (qi::_1 >= 0 && qi::_1 < 5)];
        list = '{' >> +(integer % ',') >> '}';
        start = '{' >> +(list % ',') >> '}';
    }
    qi::rule<Iterator, int(), Skipper> integer;
    qi::rule<Iterator, std::vector<int>(), Skipper> list;
    qi::rule<Iterator, std::vector<std::vector<int>>(), Skipper> start;
};

template <typename Iterator>
std::vector<std::vector<int>> parseListOfLists(Iterator first, Iterator last)
{
    std::vector<std::vector<int>> v;
    list_list_int_parser<decltype(first)> parser;
    if (!qi::phrase_parse(first, last, parser, qi::blank, v))
    {
        v.clear();
    }
    return v;
}

// #############################################
// ### BNF parser:                           ###
// ### vector<float> = {1.0,2.0,3.0,4.0,5.0} ###
// #############################################

template <typename Iterator, typename Skipper = qi::blank_type>
struct vec5f_parser : qi::grammar<Iterator, std::vector<float>(), Skipper>
{
    vec5f_parser()
        : vec5f_parser::base_type(start)
    {
        start = '{' >> qi::repeat(4)[(qi::float_ >> ',')] >> qi::float_ >> '}';
    }
    qi::rule<Iterator, std::vector<float>(), Skipper> start;
};

template <typename Iterator>
std::vector<float> parseVec5f(Iterator first, Iterator last)
{
    std::vector<float> v;
    vec5f_parser<decltype(first)> parser;
    if (!qi::phrase_parse(first, last, parser, qi::blank, v))
    {
        v.clear();
    }
    return v;
}

static std::vector<float> parseVec5f(const std::string& dimensions)
{
    return parseVec5f(dimensions.begin(), dimensions.end());
}

using string_hash::operator"" _hash;

// ###################################
// ### IO: drishti::eye::EyeModel  ###
// ###################################

static void load(const std::string& sEye, drishti::eye::EyeModel& eye)
{
    std::string sExt;
    if (sEye.find(".eye.xml") != std::string::npos)
    {
        sExt = ".eye.xml";
    }
    else
    {
        sExt = bfs::path(sEye).extension().string();
    }

    switch (string_hash::hash(sExt))
    {
        case ".eye.xml"_hash:
            eye.read(sEye);
            break;

        case ".json"_hash:
            loadJSON(sEye, eye);
            break;
    }
}

using IntVecVec = std::vector<std::vector<int>>;

static std::vector<drishti::rcpr::Recipe> createDefaultCascadeRecipes(const IntVecVec& phi)
{
    std::vector<drishti::rcpr::CPR::CprPrm::Recipe> recipes(phi.size());
    for (int i = 0; i < recipes.size(); i++)
    {
        recipes[i].paramIndex = phi[i];
    }
    return recipes;
}

static int writeDefaultRecipe(const std::string& filename, const std::string& dimensions)
{
    auto phi = parseListOfLists(dimensions.begin(), dimensions.end());
    CV_Assert(phi.size() > 0);
    for (auto& i : phi)
    {
        CV_Assert(i.size() > 0);
    }

    const auto recipes = createDefaultCascadeRecipes(phi);
    cpr::saveJSON(filename, recipes);
    return 0;
}

DRISHTI_END_NAMESPACE(cpr)
