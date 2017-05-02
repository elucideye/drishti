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
#include "drishti/core/LazyParallelResource.h"
#include "drishti/core/Line.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/geometry/motion.h"

// clang-format off
#if DRISHTI_SERIALIZE_WITH_BOOST
# include "drishti/core/boost_serialize_common.h" // optional
#endif
// clang-format on

// clang-format off
#if DRISHTI_SERIALIZE_WITH_CEREAL
# include "drishti/core/drishti_cereal_pba.h" // optional
#endif
// clang-format on

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include "videoio/VideoSourceCV.h"
#include "videoio/VideoSourceStills.h"

// Package includes:
#include "cxxopts.hpp"

#include <opencv2/highgui.hpp>

#include <cereal/types/map.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/string.hpp>
#include <cereal/archives/json.hpp>

#include <type_traits>

using AcfPtr = std::unique_ptr<drishti::acf::Detector>;

using Landmarks5 = std::array<cv::Point2f, 5>;
struct FilenameAndLandmarks
{
    std::string filename;
    std::vector<Landmarks5> landmarks;
};

using FaceLandmarks = std::map<std::string, std::vector<Landmarks5>>;
using Landmarks5Vec = std::vector<Landmarks5>;
using RectVec = std::vector<cv::Rect>;

static FaceLandmarks parseFaceData(const std::string& sInput);
static bool writeAsJson(const std::string& filename, const std::vector<cv::Rect>& objects);
static void drawObjects(cv::Mat& canvas, const std::vector<cv::Rect>& objects);
static cv::Rect2f operator*(const cv::Rect2f& roi, float scale);
static void chooseBest(std::vector<cv::Rect>& objects, std::vector<double>& scores);
static std::vector<cv::Mat> cropNegatives(const cv::Mat& I,
    const cv::Size& winSize,
    const int pad, const RectVec& objects,
    const Landmarks5Vec& landmarks);

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
    Resizer(cv::Mat& image, const cv::Size& winSize, int width = -1)
    {
        if ((width >= 0) && !image.empty())
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

    void operator()(std::vector<cv::Rect>& objects) const
    {
        if (scale != 1.f)
        {
            for (auto& o : objects)
            {
                o = cv::Rect2f(o) * (1.f / scale);
            }
        }
    }
    operator cv::Mat() { return reduced; }

    float scale = 1.f;
    cv::Mat reduced;
};

int drishti_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-acf");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput, sModel, sTruth;
    int threads = -1;
    bool doScoreLog = false;
    bool doAnnotation = false;
    bool doPositiveOnly = false;
    bool doNegatives = false; // generate negative training samples
    bool doArchiveTranslation = false;
    bool doSingleDetection = false;
    bool doWindow = false;
    bool doNms = false;
    double cascCal = 0.0;
    int cropPad = 0;
    int minWidth = -1; // minimum object width
    int maxWidth = -1; // maximum object width TODO

    cxxopts::Options options("drishti-acf", "Command line interface for ACF object detection (see Piotr's toolbox)");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("m,model", "Input file", cxxopts::value<std::string>(sModel))
        ("n,nms", "Non maximum suppression", cxxopts::value<bool>(doNms))
        ("l,min", "Minimum object width (lower bound)", cxxopts::value<int>(minWidth))
        ("c,calibration", "Cascade calibration", cxxopts::value<double>(cascCal))
        ("a,annotate", "Create annotated images", cxxopts::value<bool>(doAnnotation))
        ("p,positive", "Limit output to positve examples", cxxopts::value<bool>(doPositiveOnly))
        ("t,threads", "Thread count", cxxopts::value<int>(threads))
        ("s,scores", "Log the filenames + max score", cxxopts::value<bool>(doScoreLog))
        ("1,single", "Single detection (max)", cxxopts::value<bool>(doSingleDetection))
    
#if defined(DRISHTI_USE_IMSHOW)
        ("w,window", "Use window preview", cxxopts::value<bool>(doWindow))
#endif

        // ### Do archive conversion ###
        ("A,archive", "Do archive translation", cxxopts::value<bool>(doArchiveTranslation))
    
        // ### Negative samples ###
        ("T,truth", "Ground truth samples", cxxopts::value<std::string>(sTruth))
        ("N,negatives", "Generate negative training samples", cxxopts::value<bool>(doNegatives))
        ("P,padding", "Output crop padding", cxxopts::value<int>(cropPad))
    
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

    if (drishti::cli::directory::exists(sOutput, ".drishti-acf"))
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
    if (sModel.empty())
    {
        logger->error() << "Must specify model file";
        return 1;
    }
    if (!drishti::cli::file::exists(sModel))
    {
        logger->error() << "Specified model file does not exist or is not readable";
        return 1;
    }

    // ::::::::::::::::::::::::::::::
    // ::: Do archive translation :::
    // ::::::::::::::::::::::::::::::

    if (doArchiveTranslation)
    {
        logger->info() << "Reserializing input archive";
        AcfPtr acf = drishti::core::make_unique<drishti::acf::Detector>(sModel);
        if (acf && acf->good())
        {
            std::string base = drishti::core::basename(sModel);
            std::string filename = sOutput + "/" + base;

#if DRISHTI_SERIALIZE_WITH_CEREAL // "cpb"
            save_cpb(filename + ".cpb", *acf);
#endif

#if DRISHTI_SERIALIZE_WITH_BOOST // "pba.z"
            save_pba_z(filename + ".pba.z", *acf);
#endif

#if DRISHTI_SERIALIZE_WITH_BOOST && DRISHTI_USE_TEXT_ARCHIVES // "txt"
            save_txt_z(filename + ".txt", *acf);
#endif
        }
        else
        {
            logger->error() << "Failed to deserialize ACF archive: " << sModel;
            return 1;
        }
        return 0;
    }

    // ### Input
    int tally = int(!sInput.empty()) + int(!sTruth.empty());
    if (tally != 1)
    {
        logger->error() << "Must specify exactly one input file or ground truth file (JSON) and not both!!!";
        return 1;
    }

    // :::::::::::::::::::::::::::::::::::::::::::::::
    // ::: Pasre images w/ optional face landmarks :::
    // :::::::::::::::::::::::::::::::::::::::::::::::

    FaceLandmarks landmarks; // ground truth map
    std::shared_ptr<VideoSourceCV> video;
    if (!sTruth.empty())
    {
        landmarks = parseFaceData(sTruth);
        std::vector<std::string> filenames;
        for (const auto& r : landmarks)
        {
            filenames.push_back(r.first);
        }
        video = std::make_shared<VideoSourceStills>(filenames);
    }
    else
    {
        video = VideoSourceCV::create(sInput);
    }

    // Allocate resource manager:

    drishti::core::LazyParallelResource<std::thread::id, AcfPtr> manager = [&]() {
        AcfPtr acf = drishti::core::make_unique<drishti::acf::Detector>(sModel);
        if (acf.get() && acf->good())
        {
            // Cofigure parameters:
            acf->setDoNonMaximaSuppression(doNms);

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
            acf.release();
        }

        return acf;
    };

    std::size_t total = 0;

    std::vector<std::pair<std::string, float>> scores;

    // Parallel loop:
    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        // Get thread specific segmenter lazily:
        auto& detector = manager[std::this_thread::get_id()];
        assert(detector);
        const auto winSize = detector->getWindowSize();

        // Load current image
        auto frame = (*video)(i);
        const auto& image = frame.image;

        //cv::imwrite("/tmp/i.png", image);

        if (!image.empty())
        {
            cv::Mat imageRGB;
            switch (image.channels())
            {
                case 1:
                    cv::cvtColor(image, imageRGB, cv::COLOR_GRAY2RGB);
                    break;
                case 3:
                    cv::cvtColor(image, imageRGB, cv::COLOR_BGR2RGB);
                    break;
                case 4:
                    cv::cvtColor(image, imageRGB, cv::COLOR_BGRA2RGB);
                    break;
            }

            std::vector<double> scores;
            std::vector<cv::Rect> objects;
            if (image.size() == winSize)
            {
                const float score = detector->evaluate(imageRGB);
                scores.push_back(score);
                objects.push_back(cv::Rect({ 0, 0 }, image.size()));
            }
            else
            {
                Resizer resizer(imageRGB, detector->getWindowSize(), minWidth);
                (*detector)(resizer, objects, &scores);
                resizer(objects);

                if (doSingleDetection)
                {
                    chooseBest(objects, scores);
                }
            }

            if (!doPositiveOnly || (objects.size() > 0))
            {
                // Construct valid filename with no extension:
                std::string base = drishti::core::basename(frame.name);
                std::string filename = sOutput + "/" + base;

                float maxScore = -1e6f;
                auto iter = std::max_element(scores.begin(), scores.end());
                if (iter != scores.end())
                {
                    maxScore = *iter;
                }

                if (doScoreLog)
                {
                    logger->info() << "SCORE: " << filename << " = " << maxScore;
                }
                else
                {
                    logger->info() << ++total << "/" << video->count() << " " << frame.name << " = " << objects.size() << "; score = " << maxScore;
                }

                if (doNegatives)
                {
                    auto iter = landmarks.find(filename); // find landmarks by name
                    if (iter != landmarks.end())
                    {
                        auto negatives = cropNegatives(image, winSize, cropPad, objects, iter->second);
                        for (int j = 0; j < negatives.size(); j++)
                        {
                            std::stringstream ss;
                            ss << filename << std::setw(2) << std::setfill('0') << j << ".png";
                            cv::imwrite(ss.str(), negatives[j]);
                        }
                    }

                    return; // don't do metadata logging
                }

                // Save detection results in JSON:
                if (!writeAsJson(filename + ".json", objects))
                {
                    logger->error() << "Failed to write: " << filename << ".json";
                }

                if (doAnnotation || doWindow)
                {
                    cv::Mat canvas = image.clone();
                    drawObjects(canvas, objects);

                    if (doAnnotation)
                    {
                        cv::imwrite(filename + "_objects.png", canvas);
                    }

#if defined(DRISHTI_USE_IMSHOW)
                    if (doWindow)
                    {
                        glfw::destroyWindow("acf");
                        glfw::imshow("acf", canvas);
                        glfw::waitKey(1);
                    }
#endif
                }
            }
        }
    };

    if (threads == 1 || threads == 0 || doWindow || !video->isRandomAccess())
    {
        auto count = video->count();
        for (int i = 0; (i < count); i++)
        {
            // Increment manually, to check for end of file:
            harness({ i, i + 1 });
            if (!video->good())
            {
                break;
            }
        }
    }
    else
    {
        cv::parallel_for_({ 0, static_cast<int>(video->count()) }, harness, std::max(threads, -1));
    }

    return 0;
}

int main(int argc, char** argv)
{

#if defined(DRISHTI_USE_IMSHOW)
    // Hack/workaround needed for continuous preview in current imshow lib
    cv::Mat canvas(480, 640, CV_8UC3, cv::Scalar(0, 255, 0));
    glfw::imshow("acf", canvas);
    glfw::waitKey(1);
    glfw::destroyWindow("acf");
#endif

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

// utility

static void chooseBest(std::vector<cv::Rect>& objects, std::vector<double>& scores)
{
    if (objects.size() > 1)
    {
        int best = 0;
        for (int i = 1; i < objects.size(); i++)
        {
            if (scores[i] > scores[best])
            {
                best = i;
            }
        }
        objects = { objects[best] };
        scores = { scores[best] };
    }
}

static FaceLandmarks parseFaceData(const std::string& sInput)
{
    FaceLandmarks landmarks;

    // Write default faces:
    std::ifstream is(sInput);
    if (is)
    {
        cereal::JSONInputArchive ia(is);
        typedef decltype(ia) Archive;
        ia(GENERIC_NVP("faces", landmarks));
    }

    return landmarks;
}

static bool writeAsJson(const std::string& filename, const std::vector<cv::Rect>& objects)
{
    std::ofstream ofs(filename);
    if (ofs)
    {
        cereal::JSONOutputArchive oa(ofs);
        typedef decltype(oa) Archive; // needed by macro
        oa << GENERIC_NVP("objects", objects);
    }
    return ofs.good();
}

static void drawObjects(cv::Mat& canvas, const std::vector<cv::Rect>& objects)
{
    for (const auto& o : objects)
    {
        cv::rectangle(canvas, o, { 0, 255, 0 }, 2, 8);
    }
}

static cv::Rect2f operator*(const cv::Rect2f& roi, float scale)
{
    return { roi.x * scale, roi.y * scale, roi.width * scale, roi.height * scale };
}

static cv::Mat cropNegative(const cv::Mat& I, const cv::Rect& roi, const cv::Size& winSize, const int pad)
{
    cv::Mat crop;

    const float s = static_cast<float>(winSize.width) / roi.width;
    cv::Matx33f T1 = transformation::translate(-roi.x, -roi.y);
    cv::Matx33f S = transformation::scale(s, s);
    cv::Matx33f T2 = transformation::translate(pad, pad);
    cv::Matx33f M = T2 * S * T1;
    cv::Size winSizePad = winSize + cv::Size(pad, pad) * 2;

    cv::warpAffine(I, crop, M.get_minor<2, 3>(0, 0), winSizePad, cv::INTER_AREA);
    return crop;
}

static std::vector<cv::Mat> cropNegatives(const cv::Mat& I,
    const cv::Size& winSize,
    const int pad,
    const RectVec& objects,
    const Landmarks5Vec& landmarks)
{
    std::vector<cv::Mat> crops;

    for (int j = 0; j < objects.size(); j++)
    {
        const auto& roi = objects[j];

        // If we have ground truth, find maximum overlap:
        float jaccard = 0.f;
        for (const auto& p : landmarks)
        {
            std::vector<cv::Point2f> points(p.size());
            std::copy(begin(p), end(p), begin(points));

            const cv::Rect truth = cv::boundingRect(points); // bounding box should be adequate

            //const float score = static_cast<float>((truth & roi).area()) / (truth | roi).area();

            // Overlap percentage may work better than jaccard index:
            const float score = static_cast<float>((truth & roi).area()) / truth.area();

            jaccard = std::max(score, jaccard);
        }

        if (((roi & cv::Rect({ 0, 0 }, I.size())).area() == roi.area()) && (jaccard < 0.25))
        {
            crops.push_back(cropNegative(I, roi, winSize, pad));
        }
    }

    return crops;
}
