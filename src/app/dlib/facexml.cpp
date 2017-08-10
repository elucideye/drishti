/*!
 @file   facexml.cpp
 @author David Hirvonen
 @brief  Convert drishti face model xml files to dlib shape_predictor "parts" file.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/face/Face.h"
#include "drishti/face/FaceIO.h"
#include "drishti/core/Logger.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/Parallel.h"

#include "landmarks/DlibXML.h"
#include "landmarks/TWO.h"

#include <cereal/archives/json.hpp>

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

#include <opencv2/highgui.hpp>
#include <boost/filesystem.hpp>

#include "cxxopts.hpp"

#include <fstream>
#include <iostream>
#include <string>

namespace bfs = boost::filesystem;

using string_hash::operator"" _hash;

struct FaceEntry
{
    std::string filename;
    cv::Rect roi;
    std::vector<cv::Point2f> points;
};

static int drishtiFaceToDlib(const std::vector<FaceEntry>& faces, std::string& sOutput);

static std::vector<cv::Point2f> readPoints(const std::string& filename)
{
    std::vector<cv::Point2f> points;
    std::ifstream input(filename);
    if (input)
    {
        for (int i = 0; i < 68; i++)
        {
            cv::Point2f p;
            input >> p.x >> p.y;
            if (!input)
            {
                break;
            }
            points.push_back(p);
        }
    }
    return points;
}

static cv::Rect readRoi(const std::string& filename)
{
    cv::Rect roi;
    std::ifstream input(filename);
    if (input)
    {
        int n = 0;
        input >> n;
        if (n > 0)
        {
            input >> roi.x >> roi.y >> roi.width >> roi.height;
        }
    }
    return roi;
}

static cv::Rect scale(const cv::Rect& roi, float s)
{
    cv::Point2f tl = roi.tl(), br = roi.br(), center = (br + tl) * 0.5f, diag = (br - tl) * 0.5f;
    return cv::Rect(center - (diag * s), center + (diag * s));
}

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("eyexml");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput, sExtension = ".eye.xml";
    int threads = -1;
    int number = 0;
    bool doDump = false;

    cxxopts::Options options("eyexml", "Convert eye files to xml");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("e,extension", "Extension", cxxopts::value<std::string>(sExtension))
        ("d,dump", "Dump imags", cxxopts::value<bool>(doDump))
        ("n,number", "Number of entries (max)", cxxopts::value<int>(number))
        ("t,threads", "Threads", cxxopts::value<int>(threads))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        logger->info("{}", options.help({ "" }));
        return 0;
    }

    if (sOutput.empty())
    {
        logger->error("Must specify output xml file");
    }

    auto filenames = drishti::cli::expand(sInput);

    auto mapping = parseTWO(""); // just load the landmark format

    std::vector<FaceEntry> faces(filenames.size());

    int counter = 0;
    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        const auto& filename = filenames[i];

        counter++; // approximate indication
        logger->info("{} / {} : {}", counter, filenames.size(), filename);

        const auto pos = filename.find(".png");
        if (pos != std::string::npos)
        {
            std::string base = filename.substr(0, pos);

            std::string sTarget = base + "_objects.jpg";
            std::string sPoints = base + ".pts";
            std::string sRoi = base + ".roi";

            // Read points
            std::vector<cv::Point2f> points = readPoints(sPoints);

            // Read roi;
            cv::Rect roi = readRoi(sRoi);
            cv::Rect padded = scale(roi, 1.5);

            // Draw the annotated image:
            cv::Mat canvas;
            if (doDump)
            {
                canvas = cv::imread(filename);
                cv::rectangle(canvas, roi, { 0, 255, 0 }, 2, 8);
                cv::rectangle(canvas, padded, { 0, 255, 255 }, 2, 8);
                for (const auto& p : points)
                {
                    cv::circle(canvas, p, 3, { 0, 255, 0 }, -1, 8);
                }
            }

            if (roi.size().area() && points.size() == 68)
            {
                std::vector<std::vector<int>*> features{
                    //&mapping.browL,
                    //&mapping.browR,
                    &mapping.eyeL,
                    &mapping.eyeR,
                    &mapping.nose,
                    &mapping.mouth
                };

                // Concatenate feature indices:
                std::vector<int> index;
                for (const auto& f : features)
                {
                    for (const auto& i : (*f))
                    {
                        if (!padded.contains(points[i]))
                        {
                            const auto slash = filename.rfind("/");
                            if (slash != std::string::npos)
                            {
                                if (doDump)
                                {
                                    std::string failure = filename.substr(slash + 1);
                                    cv::imwrite("/tmp/fail/" + failure, canvas);
                                }
                                return;
                            }
                        }
                    }
                }

                // If we reach here, landmarks are ok:
                if (doDump)
                {
                    cv::imwrite(sTarget, canvas);
                }

                faces[i] = { filename, padded, points };
            }
        }
    };

    if (threads == 1 || threads == 0)
    {
        harness({ 0, static_cast<int>(filenames.size()) });
    }
    else
    {
        cv::parallel_for_({ 0, static_cast<int>(filenames.size()) }, harness, std::max(threads, -1));
    }

    if (number > 0)
    {
        auto pruner = [&](const FaceEntry& face) {
            return face.filename.empty();
        };
        faces.erase(std::remove_if(faces.begin(), faces.end(), pruner), faces.end());
        std::random_shuffle(faces.begin(), faces.end());
        if (number < faces.size())
        {
            faces.erase(faces.begin() + number, faces.end());
        }
    }

    return drishtiFaceToDlib(faces, sOutput);
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

static int drishtiFaceToDlib(const std::vector<FaceEntry>& faces, std::string& sOutput)
{
    std::ofstream os(sOutput);
    if (os)
    {
        DlibDocument doc;

        doc.start();
        for (const auto& face : faces)
        {
            doc.addPoints(face.roi, face.points, face.filename);
        }
        doc.finish();
        doc.write(os);
    }

    return 0;
}
