/*!
 @file   pose.cpp
 @author David Hirvonen
 @brief  OpenCV object detector.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/face/FaceLandmarkMeshMapper.h"
#include "drishti/face/FaceMesh.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimatorDEST.h"
#include "drishti/ml/ObjectDetectorCV.h"
#include "drishti/core/Logger.h"
#include "drishti/core/string_utils.h"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "cxxopts.hpp"

#include <iostream>
#include <sstream>
#include <numeric>

using Landmarks = std::vector<cv::Point2f>;

static Landmarks testTransform(const Landmarks& landmarks);
static void testMeshAlign(const cv::Mat& input, cv::Mat& output, drishti::face::FaceMesh& mesh, const Landmarks& landmarks);

int drishti_main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    auto logger = drishti::core::Logger::create("eos-pose");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sModel;
    std::string sMapping;
    std::string sRegressor;
    std::string sDetector;
    std::string sTriangles;
    std::string sInput;
    std::string sOutput;
    bool verbose = false;
    bool doPreview = false;
    int threads = -1;
    int width = 256;

    cxxopts::Options options("eos-pose", "Command line interface for eos 2d->3d face model fitting");

    // clang-format off
    options.add_options()("i,input", "Input file", cxxopts::value<std::string>(sInput))("o,output", "Output directory", cxxopts::value<std::string>(sOutput))

        ("w,width", "Width", cxxopts::value<int>(width))("v,verbose", "verbose", cxxopts::value<bool>(verbose))

#if defined(DRISHTI_USE_IMSHOW)
            ("p,preview", "preview", cxxopts::value<bool>(doPreview))
#endif

                ("d,detector", "detector", cxxopts::value<std::string>(sDetector))("r,regressor", "regressor", cxxopts::value<std::string>(sRegressor))("3,triangles", "delaunay triangles", cxxopts::value<std::string>(sTriangles))

                    ("m,model", "3D dephormable model", cxxopts::value<std::string>(sModel))("l,mapping", "Landmark mapping", cxxopts::value<std::string>(sMapping))

                        ("t,threads", "Thread count", cxxopts::value<int>(threads))("h,help", "Print help message");
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

    if (sInput.empty())
    {
        logger->error() << "Must specify input filename";
        return 1;
    }

    cv::Mat input = cv::imread(sInput);
    if (input.empty())
    {
        std::cerr << "Unable to read input file " << sInput << std::endl;
        return 1;
    }

    cv::resize(input, input, { width, input.rows * width / input.cols }, cv::INTER_CUBIC);

    // ########## FACE MESH LANDMARKER #########
    if (sModel.empty())
    {
        logger->error() << "Must specify 3D model";
        return 1;
    }

    if (sMapping.empty())
    {
        logger->error() << "Must specify landmark mapping";
        return 1;
    }

    drishti::face::FaceLandmarkMeshMapper mapper(sModel, sMapping);

    // ######### LANDMARK ######################
    if (sDetector.empty())
    {
        logger->error() << "Must specify detector file";
        return 1;
    }

    auto detector = std::make_shared<drishti::ml::ObjectDetectorCV>(sDetector);
    detector->setMinNeighbors(3);

    if (sRegressor.empty())
    {
        logger->error() << "Must specify regressor file";
        return 1;
    }

    auto landmarker = std::make_shared<drishti::ml::RegressionTreeEnsembleShapeEstimatorDEST>(sRegressor);

    auto faceMesh = std::make_shared<drishti::face::FaceMesh>();
    if (!sTriangles.empty())
    {
        // Load triangles (else they will be computed on the fly)
        faceMesh->readTriangulation(sTriangles);
    }

    Landmarks landmarks;
    if (detector && landmarker)
    {
        cv::Mat gray;
        cv::extractChannel(input, gray, 1);

        cv::Rect face;

        { // Detect largest face via OpenCV (used for DEST training):
            std::vector<cv::Rect> faces;
            (*detector)(gray, faces);
            if (faces.size())
            {
                std::sort(begin(faces), end(faces), [](const cv::Rect& a, const cv::Rect& b) { return a.area() > b.area(); });
                face = faces.front();
            }
            else
            {
                logger->info() << "No faces found";
                return 0;
            }
        }

        { // Get landmarks:
            std::vector<bool> mask;
            (*landmarker)(gray(face), landmarks, mask);
            for (auto& p : landmarks)
            {
                p += cv::Point2f(face.x, face.y);
            }
        }

        if (faceMesh && landmarks.size())
        {
            cv::Mat output;
            testMeshAlign(input, output, *faceMesh, landmarks);
        }

        std::string sBase = drishti::core::basename(sInput);
        if (!sOutput.empty() || doPreview)
        {
            cv::Mat canvas = input.clone();
            faceMesh->draw(canvas, landmarks, faceMesh->delaunay(landmarks, canvas.size()));

            if (sOutput.empty())
            {
                cv::imwrite(sOutput + "/" + sBase + "_annotation.png", canvas);
            }

#if defined(DRISHTI_USE_IMSHOW)
            if (doPreview)
            {
                glfw::imshow("triangles", canvas);
                glfw::waitKey(0);
            }
#endif

            faceMesh->writeTriangulation(sOutput + "/triangles.yaml");
        }

        if (!sMapping.empty() && !sModel.empty() && landmarks.size())
        {
            cv::Mat iso;
            eos::render::Mesh mesh;
            cv::Point3f R = mapper(landmarks, input, mesh, iso);

            std::cout << "R = " << R << std::endl;

            // (((( Draw mesh for visualization ))))
            if (doPreview || !sOutput.empty())
            {
                mapper.draw(iso, mesh);

                for (auto& p : mesh.texcoords)
                {
                    p[0] *= iso.cols;
                    p[1] *= iso.rows;
                }

                for (int i = 0; i < mesh.tvi.size(); i++)
                {
                    const auto& t = mesh.tvi[i];
                    cv::Point2f v0 = mesh.texcoords[t[0]];
                    cv::Point2f v1 = mesh.texcoords[t[1]];
                    cv::Point2f v2 = mesh.texcoords[t[2]];
                    cv::line(iso, v0, v1, { 0, 255, 0 }, 1, 8);
                    cv::line(iso, v1, v2, { 0, 255, 0 }, 1, 8);
                    cv::line(iso, v2, v0, { 0, 255, 0 }, 1, 8);
                }

                if (!sOutput.empty())
                {
                    cv::imwrite(sOutput + "/" + sBase + ".png", iso);
                }

#if defined(DRISHTI_USE_IMSHOW)
                if (doPreview)
                {
                    glfw::imshow("iso", iso);
                    glfw::waitKey(0);
                }
#endif
            }
        }
    }
}

int main(int argc, char** argv)
{
    try
    {
        return drishti_main(argc, argv);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

static Landmarks testTransform(const Landmarks& landmarks)
{
    Landmarks landmarks2 = landmarks;

    const float theta = 45.0;
    const float ct = std::cos(theta * M_PI / 180.0);
    const float st = std::sin(theta * M_PI / 180.0);
    cv::Point2f center = landmarks[0];

    cv::Matx33f T(1, 0, center.x, 0, 1, center.y, 0, 0, 1);
    cv::Matx33f R(+ct, -st, 0.0, +st, +ct, 0.0, 0.0, 0.0, 1.0);
    cv::Matx33f H = T * R * T.inv();

    for (auto& p : landmarks2)
    {
        cv::Point3f q = H * cv::Point3f(p.x, p.y, 1.f);
        p = { q.x / q.z, q.y / q.z };
    }

    return landmarks2;
}

static void testMeshAlign(const cv::Mat& input, cv::Mat& output, drishti::face::FaceMesh& mesh, const Landmarks& landmarks)
{
    auto landmarks2 = testTransform(landmarks);
    auto map = mesh.transform(landmarks2, landmarks, input.size());

    cv::Mat flow, canvas;
    cv::hconcat(map[0], map[1], flow);
    cv::normalize(flow, canvas, 0, 255, cv::NORM_MINMAX, CV_8UC1);
    cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);

    cv::Mat warped;
    cv::remap(input, warped, map[0], map[1], cv::INTER_LINEAR);
    cv::hconcat(warped, canvas, canvas);

#if defined(DRISHTI_USE_IMSHOW)
    glfw::imshow("remaped", canvas);
    glfw::waitKey(0);
#endif
}
