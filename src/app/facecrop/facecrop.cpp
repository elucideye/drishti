/*!
  @file   facecrop.cpp
  @author David Hirvonen
  @brief  Batch processing to parse and synthesize new face images from various databases.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

 
 1) POSITIVES
    --input=<FILE> : filename + (optional) landmarks
    --format=(two|drishti|lfw|muct|helen|bioid|lfpw)
    --output=<POSITIVE_DIR>

 2) POSITIVES + NEGATIVES (non-overlapping negatives) 
    :: This would require adding object detection internally, it seems cleaner to leave hard negative
    :: sampling to another application.
    --input=<FILE> : filename + (optional) landmarks
    --format=(two|drishti|lfw|muct|helen|bioid|lfpw)
    --output=<POSITIVE_DIR>
    --negatives=<NEGATIVE_DIR>

 3) NEGATIVES ONLY (raw input file list containing no faces)
    --input=<FILE> : filename + (optional) landmarks
    --format=raw
    --negatives=<NEGATIVE_DIR>
 
 4) NEGATIVES w/ INPAINTING
    --input=<FILE> : filename + (optional) landmarks
    --format=raw
    --inpaint
    --background=<FILE> : background image list (no positives)
    --negatives=<NEGATIVE_DIR>

*/

#include "drishti/core/drishti_stdlib_string.h" // android workaround
#include "drishti/core/Logger.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/LazyParallelResource.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/core/string_utils.h"
#include "drishti/geometry/motion.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/face/Face.h" // for face model

// clang-format off
#if defined(DRISHTI_BUILD_EOS)
#  include "drishti/face/FaceMeshMapperLandmark.h"
#endif
// clang-format on

#include "landmarks/FACE.h"
#include "landmarks/MUCT.h"
#include "landmarks/HELEN.h"
#include "landmarks/BIOID.h"
#include "landmarks/LFW.h"
#include "landmarks/LFPW.h"
#include "landmarks/DRISHTI.h"
#include "landmarks/TWO.h"

#include "FaceSpecification.h"
#include "FaceJitterer.h"
#include "Pyramid.h"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <cereal/archives/json.hpp>
#include <cereal/archives/xml.hpp>
#include <cereal/types/array.hpp>
#include <cereal/types/map.hpp>

#include "cxxopts.hpp"

#include <fstream>

enum GroundTruthFormat
{
    MUCTFormat,
    FDDBFormat,
    HELENFormat,
    BIOIDFormat,
    LFWFormat,
    DRISHTIFormat,
    LFPWFormat,
    TWOFormat,
    RAWFormat
};

#define SUPPORTED_FORMATS "muct,fddb,helen,bioid,lfw,drishti,lfpw,two"

struct GroundTruth
{
    GroundTruth() {}
    GroundTruth(const FACE::Table& table, GroundTruthFormat format)
        : table(table)
        , format(format)
    {
    }

    FACE::Table table;
    GroundTruthFormat format;
};

// ######################### SHA1 #####################################
#include <cstdio>
#include <string>
#include <boost/uuid/sha1.hpp>

std::string get_sha1(void const* buffer, std::size_t byte_count)
{
    boost::uuids::detail::sha1 sha1;
    sha1.process_bytes(buffer, byte_count);
    unsigned hash[5] = { 0 };
    sha1.get_digest(hash);

    // Back to string
    char buf[41] = { 0 };

    for (int i = 0; i < 5; i++)
    {
        std::sprintf(buf + (i << 3), "%08x", hash[i]);
    }

    return std::string(buf);
}

/// ######################## >> FACE << #############################

struct FaceJittererMean : public FaceJitterer
{
    FaceJittererMean(const FACE::Table& table, const JitterParams& params, const FaceSpecification& face)
        : FaceJitterer(table, params, face)
    {
    }

    void updateMean(const std::vector<FaceWithLandmarks>& faces)
    {
        for (const auto& f : faces)
        {
            mu.updateMean(f);
        }
    }

    FaceWithLandmarksMean mu;
};

using ImageVec = std::vector<cv::Mat>;
using FaceJittererMeanPtr = std::unique_ptr<FaceJittererMean>;
using FaceResourceManager = drishti::core::LazyParallelResource<std::thread::id, FaceJittererMeanPtr>;
static int saveNegatives(const FACE::Table& table, const std::string& sOutput, int sampleCount, int winSize, int threads, spdlog::logger& logger);
static int saveInpaintedSamples(const FACE::Table& table, const std::string sBackground, const std::string& sOutput, spdlog::logger& logger);
static FaceWithLandmarks computeMeanFace(FaceResourceManager& manager);
static void saveMeanFace(FaceResourceManager& manager, const FaceSpecification& faceSpec, const std::string& sImage, const std::string& sPoints, spdlog::logger& logger);
static int saveDefaultConfigs(const std::string& sOutput, spdlog::logger& logger);
static void save(const std::vector<FaceWithLandmarks>& faces, const cv::Rect& roi, const std::string& dir, const std::string& filename, int index);
static void previewFaceWithLandmarks(cv::Mat& image, const std::vector<cv::Point2f>& landmarks);
static GroundTruth parseInput(const std::string& sInput, const std::string& sFormat, const std::string& sDirectoryIn, const std::string& sExtension);
static FACE::Table parseRAW(const std::string& sInput);
static int standardizeFaceData(const FACE::Table& table, const std::string& sOutput);

#if defined(DRISHTI_BUILD_EOS)
// Face pose estimation...
using FaceMeshMapperPtr = std::unique_ptr<drishti::face::FaceMeshMapperLandmark>;
using FaceMeshMapperResourceManager = drishti::core::LazyParallelResource<std::thread::id, FaceMeshMapperPtr>;
static void computePose(FACE::Table& table, const std::string& sModel, const std::string& sMapping, std::shared_ptr<spdlog::logger>& logger);
#endif // DRISHTI_BUILD_POSE

int gauze_main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-facecrop");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput;
    std::string sFormat;
    std::string sPositives;
    std::string sNegatives;
    std::string sDirectory;
    std::string sExtension;
    std::string sBackground;
    std::string sStandardize;
    std::string sFaceSpec;
    std::string sJitterIn;

#if defined(DRISHTI_BUILD_EOS)
    std::string sEosModel;
    std::string sEosMapping;
#endif

    int sampleCount = 0;
    int winSize = 48; // min crop width
    int threads = -1;
    bool doInpaint = false;
    bool doPreview = false;
    bool doBoilerplate = false;
    bool doPhotometricJitterOnly = false;

    cxxopts::Options options("drishti-facecrop", "Command line interface for facecrop object detection.");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("p,positives", "Positives directory", cxxopts::value<std::string>(sPositives))
        ("f,format", "Format:" SUPPORTED_FORMATS, cxxopts::value<std::string>(sFormat))
        ("d,directory", "Base (d)irectory", cxxopts::value<std::string>(sDirectory))
        ("s,specification", "Face specification", cxxopts::value<std::string>(sFaceSpec))
        ("j,jitter", "Jitter input parameters", cxxopts::value<std::string>(sJitterIn))
        ("n,number", "Number of output samples to generate", cxxopts::value<int>(sampleCount))
        ("b,boilerplate", "Write boilerplate config to output dir", cxxopts::value<bool>(doBoilerplate))
        ("e,extension", "Image filename extensions", cxxopts::value<std::string>(sExtension))
        ("w,window", "Do preview window", cxxopts::value<bool>(doPreview))
        ("0,zero", "Zero jitter model (photometric jitter only)", cxxopts::value<bool>(doPhotometricJitterOnly))

#if defined(DRISHTI_BUILD_EOS)
        ("eos-model", "EOS 3D dephormable model", cxxopts::value<std::string>(sEosModel))
        ("eos-mapping", "EOS Landmark mapping", cxxopts::value<std::string>(sEosMapping))
#endif
    
        ("standardize", "Standardize input files to filename + bounding boxes", cxxopts::value<std::string>(sStandardize))
    
        // ### Negative options ###
        ("I,inpaint", "Inpaint faces", cxxopts::value<bool>(doInpaint))
        ("N,negatives","Negatives", cxxopts::value<std::string>(sNegatives))
        ("W,winsize", "Minimum window size", cxxopts::value<int>(winSize))
        ("B,background", "Background image list", cxxopts::value<std::string>(sBackground))
    
        // Output parameters:
        ("t,threads", "Thread count", cxxopts::value<int>(threads))
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

    // ### Directory
    if(sPositives.empty() && sNegatives.empty())
    {
        logger->error("Must specify output directory (positives or negatives)");
        return 1;
    }
    
    // ... positives ...
    if(!sPositives.empty())
    {
        if(drishti::cli::directory::exists(sPositives, ".drishti-facecrop"))
        {
            std::string filename = sPositives + "/.drishti-facecrop";
            remove(filename.c_str());
        }
        else
        {
            logger->error("Specified directory {} does not exist or is not writeable", sPositives);
            return 1;
        }
    }

    // ... negatives ...
    if(!sNegatives.empty())
    {
        if(drishti::cli::directory::exists(sNegatives, ".drishti-facecrop"))
        {
            std::string filename = sNegatives + "/.drishti-facecrop";
            remove(filename.c_str());
        }
        else
        {
            logger->error("Specified directory {} does not exist or is not writeable", sNegatives);
            return 1;
        }
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
    
    //:::::::::::::::::::::::::::::::
    //::: Parse input + landmarks :::
    //:::::::::::::::::::::::::::::::
    GroundTruth gt = parseInput(sInput, sFormat, sDirectory, sExtension);
    auto &table = gt.table;
    
    if(table.lines.empty())
    {
        logger->error("Error: no images were found, please check input file and (optionally) base directory");
        return -1;
    }
    else
    {
        // Try simple image read sanity test for user feedback:
        if(cv::imread(table.lines.front().filename).empty())
        {
            logger->error("Error: unable to read input image, please check input file and (optionally) base directory");
            return -1;
        }
    }
    
    if(!sStandardize.empty())
    {
        return standardizeFaceData(table, sStandardize);
    }

    if(doBoilerplate)
    {
        if(int code = saveDefaultConfigs(sPositives, *logger) < 0)
        {
            return code;
        }
    }
    
    if(sPositives.empty() && !sNegatives.empty() && !doInpaint)
    {
        // ##########################
        // ### 3) NEGATIVES ONLY  ### >>> Sample random negative windows and quit <<<
        // ##########################
        return saveNegatives(table, sNegatives, sampleCount, winSize, threads, *logger);
    }

    if(doInpaint && !sBackground.empty() && !sNegatives.empty())
    {
        // ###################################
        // ### 4) NEGATIVES w/ INPAINTING  ###
        // ###################################
        return saveInpaintedSamples(table, sBackground, sNegatives, *logger);
    }
    
    // ... ELSE STANDARD POSITIVES AND/OR NEGATIVES ...
    
    //::::::::::::::::::::::::::
    //::: Face jitter params :::
    //::::::::::::::::::::::::::
    JitterParams jitterParams;
    if(!sJitterIn.empty())
    {
        std::ifstream is(sJitterIn);
        if(is)
        {
            cereal::JSONInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia(GENERIC_NVP("jitter", jitterParams));
        }
        else
        {
            logger->error("Error: unable to read input jitter parameters");
            return -1;
        }
    }
    
    //:::::::::::::::::::::::::::::::::
    //::: Face normalization params :::
    //:::::::::::::::::::::::::::::::::
    FaceSpecification faceSpec;
    if(sFaceSpec.empty())
    {
        logger->error("Error: must provide valid face specification");
        return -1;
    }
    else
    {
        std::ifstream is(sFaceSpec);
        if(is)
        {
            cereal::JSONInputArchive ia(is);
            typedef decltype(ia) Archive;
            ia(GENERIC_NVP("face", faceSpec));
        }
        else
        {
            logger->error("Error: unable to read face specification file: {}", sFaceSpec);
            return -1;
        }
    }
    
    
#if defined(DRISHTI_BUILD_EOS)
    if(!(sEosModel.empty() || sEosMapping.empty()))
    {
        computePose(table, sEosModel, sEosMapping, logger);
        auto posePruner = [&](FACE::record &record)
        {
            const bool doPrune = (std::abs(record.pose.y) > 40.f) || (std::abs(record.pose.z) > 40.f);
            if(doPrune)
            {
                logger->info("prune: {}", record.pose);
            }
            return doPrune;
        };
        
        auto &lines = table.lines;
        lines.erase(std::remove_if(lines.begin(), lines.end(), posePruner), lines.end());
    }
#endif

    // Determine samples:
    std::vector<int> repeat(table.lines.size(), 1);
    if(sampleCount > 0)
    {
        std::fill(begin(repeat), end(repeat), 0);
        std::vector<int> samples(sampleCount);
        cv::RNG().fill(samples, 0, 0, table.lines.size());
        for(const auto &i : samples)
        {
            repeat[i]++;
        }
    }

    FaceResourceManager manager = [&]()
    {
        logger->info("Create resource...");
        return drishti::core::make_unique<FaceJittererMean>(table, jitterParams, faceSpec);
    };

    // ####################
    // ### 1) POSITIVES ###
    // ####################
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto tid = std::this_thread::get_id();
        
        auto &jitterer = manager[tid];
        assert(jitterer.get());
        
        // Load current image
        logger->info("{} = {}", table.lines[i].filename, repeat[i]);
        
        if(repeat[i] > 0)
        {
            cv::Mat image = cv::imread(table.lines[i].filename, cv::IMREAD_COLOR);
            
            if(!image.empty())
            {
                std::vector<FaceWithLandmarks> faces { (*jitterer)(image, table.lines[i].points, false, true) };
                for(int j = 1; j < repeat[i]; j++)
                {
                    faces.push_back((*jitterer)(image, table.lines[i].points, !doPhotometricJitterOnly, true));
                }
                
                jitterer->updateMean(faces);

                if(!sPositives.empty())
                {
                    cv::Rect roi(cv::Point(faceSpec.border, faceSpec.border), faceSpec.size);
                    save(faces, roi, sPositives, table.lines[i].filename, i);
                }
                
#if defined(DRISHTI_USE_IMSHOW)
                if(doPreview)
                {
                    cv::Mat canvas = image.clone();
                    previewFaceWithLandmarks(canvas, table.lines[i].points);
                    glfw::imshow("facecrop:image", canvas);
                    
                    std::vector<cv::Mat> images;
                    for(const auto &f : faces)
                    {
                        images.push_back(f.image);
                    }
                    
                    cv::hconcat(images, canvas);
                    glfw::imshow("facecrop:jitter", canvas);
                    glfw::imshow("facecrop::mu", jitterer->mu.image);
                    
                    glfw::waitKey(0);
                }
#endif
            }
        }
    };
    
    if(threads == 1 || threads == 0 || doPreview)
    {
        harness({0,static_cast<int>(table.lines.size())});
    }
    else
    {
        cv::parallel_for_({0,static_cast<int>(table.lines.size())}, harness, std::max(threads, -1));
    }

    saveMeanFace(manager, faceSpec, sPositives + "/mean.png", sPositives + "/mean", *logger);
    
    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch(cv::Exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    catch (std::exception &e)
    {
        std::cerr << e.what() << std::endl;
    }
    catch(...)
    {
        std::cerr << "Unknown exception catched" << std::endl;
    }
    
    exit(-1);
}


// ### utility ###

using string_hash::operator "" _hash;
static GroundTruth parseInput(const std::string &sInput, const std::string &sFormat, const std::string &sDirectoryIn, const std::string &sExtension)
{
    GroundTruth gt;
    switch(string_hash::hash(sFormat))
    {
        case "two"_hash:
            gt.format = TWOFormat;
            gt.table = parseTWO(sInput);
            break;
        case "drishti"_hash:
            gt.format = DRISHTIFormat;
            gt.table = parseDRISHTI(sInput);
            break;
        case "lfw"_hash :
            gt.format = LFWFormat;
            gt.table = parseLFW(sInput);
            break;
        case "muct"_hash  :
            gt.format = MUCTFormat;
            gt.table = parseMUCT(sInput);
            break;
        case "helen"_hash :
            gt.format = HELENFormat;
            gt.table = parseHELEN(sInput);
            break;
        case "bioid"_hash :
            gt.format = BIOIDFormat;
            gt.table = parseBIOID(sInput);
            break;
        case "lfpw"_hash :
            gt.format = LFPWFormat;
            gt.table = parseLFPW(sInput);
            break;
        case "raw"_hash :
            gt.table = parseRAW(sInput);
            gt.format = RAWFormat;
            break;
        default :
            CV_Assert(false);
    }
    
    std::string sDirectory = sDirectoryIn;
    if(!sDirectory.empty())
    {
        if(sDirectory.back() != '/')
        {
            sDirectory += "/";
        }
        
        for(auto &l : gt.table.lines)
        {
            l.filename = sDirectory + l.filename;
            if(!sExtension.empty())
            {
                l.filename += ".";
                l.filename += sExtension;
            }
        }
    }
    for(auto &l : gt.table.lines)
    {
        std::replace(begin(l.filename), end(l.filename), '\\', '/');
    }
    
    return gt;
}


static FACE::Table parseRAW(const std::string &sInput)
{
    const auto filenames = drishti::cli::expand(sInput);
    
    FACE::Table table;
    table.lines.resize(filenames.size());
    for(int i = 0; i < filenames.size(); i++)
    {
        table.lines[i].filename = filenames[i];
    }
    return table;
}

static std::map<std::string, std::vector<std::array<cv::Point2f, 5>>> standardizeFaceData(const FACE::Table &table)
{
    std::map<std::string, std::vector<std::array<cv::Point2f, 5>>> landmarks;
    for(const auto &r : table.lines)
    {
        std::array<cv::Point2f, 5> points
        {{
            r.points[table.eyeR.front()],
            r.points[table.eyeL.front()],
            r.points[table.nose.front()],
            r.points[table.mouthR.front()],
            r.points[table.mouthL.front()]
        }};
        landmarks[r.filename].emplace_back(points);
    }
    return landmarks;
}

static int standardizeFaceData(const FACE::Table &table, const std::string &sOutput)
{
    // Write default faces:
    std::ofstream os(sOutput);
    if(os)
    {
        auto landmarks = standardizeFaceData(table);
        
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("faces", landmarks));
    }
    else
    {
        return -1;
    }
    return 0;
}

#if defined(DRISHTI_BUILD_EOS)

#include <fstream>
#include <iostream>

// http://stackoverflow.com/a/22926477
template <typename T>
inline T ntoh_any(T t)
{
    static const unsigned char int_bytes[sizeof(int)] = {0xFF};
    static const int msb_0xFF = 0xFF << (sizeof(int) - 1) * CHAR_BIT;
    static bool host_is_big_endian = (*(reinterpret_cast<const int *>(int_bytes)) & msb_0xFF ) != 0;
    if (host_is_big_endian) { return t; }

    unsigned char * ptr = reinterpret_cast<unsigned char *>(&t);
    std::reverse(ptr, ptr + sizeof(t) );
    return t;
}

static cv::Size read_png_size(const std::string &filename)
{
    cv::Size size; // 0
    std::ifstream in(filename);
    if(in)
    {
        unsigned int width, height;
        in.seekg(16);
        in.read((char *)&width, 4);
        in.read((char *)&height, 4);
        size = { static_cast<int>(ntoh_any(width)), static_cast<int>(ntoh_any(height)) };        
    }
    return size;
}

static void computePose(FACE::Table &table, const std::string &sModel, const std::string &sMapping, std::shared_ptr<spdlog::logger> &logger)
{
    FaceMeshMapperResourceManager manager = [&]()
    {
        return drishti::core::make_unique<drishti::face::FaceMeshMapperLandmark>(sModel, sMapping);
    };
    
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        // Get thread specific segmenter lazily:
        auto tid = std::this_thread::get_id();
        auto &meshMapper = manager[tid];
        
        auto &record = table.lines[i];
        record.pose = {180.f, 180.f, 180.f };
        if(record.points.size() == 68)
        {
            cv::Size size;

            if((record.filename.find(".png") != std::string::npos) || (record.filename.find(".PNG") != std::string::npos))
            {
                size = read_png_size(record.filename);
            }
            if(size.area() == 0)
            {
                size = cv::imread(record.filename).size();
            }
            
            eos::core::Mesh mesh;

            cv::Mat dummy;
            dummy.cols;
            dummy.rows;
            auto result = (*meshMapper)(record.points, dummy);
            record.pose = drishti::face::getRotation(result.rendering_params);

            logger->info("{} {} {}", record.filename, size, record.pose);
        }
    };
    
    cv::parallel_for_({0,static_cast<int>(table.lines.size())}, harness, 8);
}
#endif 

static int saveLandmarksJson(const std::string &sOutput, const std::array<cv::Point2f, 5> &landmarks, spdlog::logger &logger)
{
    // Write default jitter parameters:
    std::ofstream os(sOutput);
    if(os)
    {
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("landmarks", landmarks));
    }
    else
    {
        logger.error("Error: unable to write mean landmarks");
        return -1;
    }
    return 0;
}

static int saveLandmarksXml(const std::string &sOutput, const drishti::face::FaceModel &landmarks, spdlog::logger &logger)
{
    // Write default jitter parameters:
    std::ofstream os(sOutput);
    if(os)
    {
        cereal::XMLOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("landmarks", landmarks));
    }
    else
    {
        logger.error("Error: unable to write mean landmarks");
        return -1;
    }
    return 0;
}

static void saveMeanFace(FaceResourceManager &manager, const FaceSpecification &faceSpec, const std::string &sImage, const std::string &sPoints, spdlog::logger &logger)
{
    // Save the mean face image:
    FaceWithLandmarks mu = computeMeanFace(manager);
    if(!sImage.empty())
    {
        cv::Mat tmp;
        mu.image.convertTo(tmp, CV_8UC3, 255.0);
        cv::imwrite(sImage, tmp);
    }
    
    if(!sPoints.empty())
    {
        // Output json for points
        saveLandmarksJson(sPoints + ".json", mu.landmarks, logger);

        // Legacy: save a mean face model structure (normalized):
        cv::Point2f tl(faceSpec.border, faceSpec.border);
        drishti::face::FaceModel model;
        model.eyeRightCenter = mu.landmarks[0];
        model.eyeLeftCenter = mu.landmarks[1];
        model.noseTip = mu.landmarks[2];
        
        cv::Matx33f T = transformation::translate(-tl);
        cv::Matx33f S = transformation::scale(faceSpec.size.width, faceSpec.size.height);
        model = (S.inv() * T) * model; // remove border and normalize
        saveLandmarksXml(sPoints + ".xml", model, logger);
    }
}

static int saveDefaultJitter(const std::string &sOutput, spdlog::logger &logger)
{
    // Write default jitter parameters:
    std::ofstream os(sOutput);
    if(os)
    {
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("jitter", JitterParams()));
    }
    else
    {
        logger.error("Error: unable to write default jitter parameters");
        return -1;
    }
    return 0;
}

static int saveDefaultFaceSpec(const std::string &sOutput, spdlog::logger &logger)
{
    // Write default face specification:
    std::ofstream os(sOutput);
    if(os)
    {
        cereal::JSONOutputArchive oa(os);
        typedef decltype(oa) Archive;
        oa(GENERIC_NVP("face", FaceSpecification()));
    }
    else
    {
        logger.error("Error: unable to write default face specification parameters");
        return -1;
    }
    return 0;
}

static int saveDefaultConfigs(const std::string &sOutput, spdlog::logger &logger)
{
    if(int code = saveDefaultJitter(sOutput  + "/jitter.json", logger) != 0)
    {
        return code;
    }
    if(int code = saveDefaultFaceSpec(sOutput + "/face.json", logger) != 0)
    {
        return code;
    }
    return 0;
}

static FaceWithLandmarks computeMeanFace(FaceResourceManager &manager)
{
    int count = 0;
    for(const auto &j : manager.getMap())
    {
        count += j.second->mu.count;
    }
 
    FaceWithLandmarks mu;
    for(const auto &j : manager.getMap())
    {
        if(!j.second->mu.image.empty())
        {
            const double w = double(j.second->mu.count)/count;
            if(mu.image.empty())
            {
                mu = (j.second->mu * w);
            }
            else
            {
                mu += (j.second->mu * w);
            }
        }
    }
    
    return mu;
}

// Save in piotr's toolbox bbGt version=3 format
//
// % bbGt version=3
// face 128 117 124 124 0 128 117 124 124 0 0

static void save_bbGtv3(const std::string &sOutput, const std::vector<cv::Rect> &objects)
{
    static const char *sHeader = "% bbGt version=3";
    static const char *sLabel = "face";
    
    std::ofstream ofs(sOutput);
    if(ofs)
    {
        for(const auto &o : objects)
        {
            std::stringstream box;
            box << o.x << ' ' << o.y << ' ' << o.width << ' ' << o.height;
            ofs << sHeader << "\n" << sLabel << ' ' << box.str() << " 0 " << box.str() << " 0 0" << std::endl;
        }
    }
}

static void save(const std::vector<FaceWithLandmarks> &faces, const cv::Rect &roi, const std::string &dir, const std::string &filename, int index)
{
    for(int i = 0; i < faces.size(); i++)
    {
        
        std::stringstream ss;
        ss << std::setfill('0') << std::setw(6) << index << "_" << std::setw(2) << i;
        std::string base = drishti::core::basename(filename);

        { // save the image file
            std::string sOutput = dir + "/" + ss.str() + "_" + base + ".png";
            cv::imwrite(sOutput, faces[i].image);
        }
        
        {  // Save bbox file for each face (compatible w/ Piotr's toolbox):
            std::string sOutput = dir + "/" + ss.str() + "_" + base + ".txt";
            save_bbGtv3(sOutput, {roi});
        }
    }
}

static void previewFaceWithLandmarks(cv::Mat &image, const std::vector<cv::Point2f> &landmarks)
{
    for(const auto &p : landmarks)
    {
        cv::circle(image, p, 2, {0,255,0}, -1, 8);
    }
}


static int saveNegatives(const FACE::Table &table, const std::string &sOutput, int sampleCount, int winSize, int threads, spdlog::logger &logger)
{
    std::vector<int> repeat(table.lines.size(), 1);
    if(sampleCount > 0)
    {
        std::fill(begin(repeat), end(repeat), 0);
        std::vector<int> samples(sampleCount);
        cv::RNG().fill(samples, 0, 0, repeat.size());
        for(const auto &i : samples)
        {
            repeat[i]++;
        }
    }

    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        cv::RNG rng;
        
        const auto &f = table.lines[i].filename;
        cv::Mat negative = cv::imread(f, cv::IMREAD_COLOR);
        
        int minDim = std::min(negative.cols, negative.rows);
        if((minDim >= winSize) && !negative.empty())
        {
            for(int j = 0; j < repeat[i]; j++)
            {
                const int width = rng.uniform(winSize, minDim);
                const int x = rng.uniform(0, negative.cols-width);
                const int y = rng.uniform(0, negative.rows-width);
                
                logger.info("roi:{},{},{},{}({})", x, y, width, width, winSize);
                cv::Mat crop = negative(cv::Rect(x, y, width, width));
                
                cv::resize(crop, crop, {winSize, winSize}, 0, 0, cv::INTER_AREA);
                std::string sha1 = get_sha1(crop.ptr<void>(), crop.total());
                cv::imwrite(sOutput + "/" + sha1 + ".png", crop);
            }
        }
    };
    
    //cv::parallel_for_({0,static_cast<int>(sInput.size())}, harness, std::max(threads, -1));
    harness({0,static_cast<int>(repeat.size())});
    
    return 0;
}


static int saveInpaintedSamples(const FACE::Table &table, const std::string sBackground, const std::string &sOutput, spdlog::logger &logger)
{
    cv::RNG rng;
    
    // Read a bunch of negative samples:
    std::vector<cv::Mat> negatives;
    auto filenames = drishti::cli::expand(sBackground);
    for(int i = 0; i < std::min(100, static_cast<int>(filenames.size())); i++)
    {
        cv::Mat I = cv::imread(filenames[rng.uniform(0, filenames.size())], cv::IMREAD_COLOR);
        if(!I.empty())
        {
            negatives.push_back(I);
        }
    }
    
    std::map< std::string, std::vector<const std::vector<cv::Point2f>*> > landmarks;
    for(const auto &r : table.lines)
    {
        landmarks[r.filename].push_back(&r.points);
    }
    
    drishti::core::ParallelHomogeneousLambda harness = [&](int i)
    {
        const auto &r = table.lines[i];
        cv::Mat image = cv::imread(r.filename, cv::IMREAD_COLOR);
        if(!image.empty())
        {
            logger.info("faceless:{}", r.filename);
            
            cv::Mat blended;
            auto iter = landmarks.find(r.filename);
            if(iter != landmarks.end())
            {
                for(const auto &p : iter->second)
                {
                    const cv::Rect roi = cv::boundingRect(*p);
                    const cv::Point2f tl = roi.tl(), br = roi.br(), center = (br + tl) * 0.5f;
                    const cv::RotatedRect face(center, cv::Size2f(roi.width, roi.height*2.f), 0);
                    cv::Mat mask(image.size(), CV_8UC3, cv::Scalar::all(0));
                    cv::ellipse(mask, face, cv::Scalar::all(255), -1, 8);
                    
                    cv::Mat bg;
                    cv::resize(negatives[rng.uniform(0, negatives.size())], bg, image.size(), 0, 0, cv::INTER_AREA);
                    
                    blended = blend(blended.empty() ? image : blended, bg, mask, 6);
                    blended.convertTo(blended, CV_8UC3, 255.0);
                }
            }
            
            if(!blended.empty())
            {
                std::string base = drishti::core::basename(r.filename);
                cv::imwrite(sOutput + "/" + base + "_faceless.png", blended);
            }
        }
    };
    
    //cv::parallel_for_({0,static_cast<int>(table.lines.size()}, harness, std::max(threads, -1));
    harness({0,static_cast<int>(table.lines.size())});
    
    return 0;
}
