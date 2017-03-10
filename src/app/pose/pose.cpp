// Copyright (c) 2016, David Hirvonen
// All rights reserved.

#include "drishti/face/FaceLandmarkMeshMapper.h"
#include "Drishti/core/Logger.h"
#include "FaceLandmarker.h"

#include "cxxopts.hpp"

#include <iostream>
#include <sstream>
#include <numeric>

const char *keys =
{
    "{ input     |       | input filename                            }"
    "{ output    |       | output filename                           }"
    
    "{ width     | 256   | processing width                          }"
    "{ verbose   | false | verbose mode (w/ display)                 }"

    // Tracker file
    "{ regressor |       | face landmark regressor file              }"
    "{ detector  |       | face detector                             }"

    "{ model     |       | model file                                }"
    "{ mapping   |       | mapping file                              }"
    
    "{ threads   | false | use worker threads when possible          }"
    "{ verbose   | false | print verbose diagnostics                 }"
    "{ build     | false | print the OpenCV build information        }"
    "{ help      | false | print help message                        }"
};

int main(int argc, char *argv[])
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
    std::string sInput;
    std::string sOutput;
    bool verbose = false;
    int threads = -1;
    int width = 256;
    
    cxxopts::Options options("eos-pose", "Command line interface for eos 2d->3d face model fitting");
    options.add_options()
    ("i,input", "Input file", cxxopts::value<std::string>(sInput))
    ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
   
    ("w,width", "Width", cxxopts::value<int>(width))
    ("v,verbose", "verbose", cxxopts::value<bool>(verbose))
    
    ("d,detector", "detector", cxxopts::value<std::string>(sDetector))
    ("r,regressor", "regressor", cxxopts::value<std::string>(sRegressor))
    
    ("m,model", "3D dephormable model", cxxopts::value<std::string>(sModel))
    ("l,mapping", "Landmark mapping", cxxopts::value<std::string>(sMapping))
    
    ("t,threads", "Thread count", cxxopts::value<int>(threads))
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
    
    if(sInput.empty())
    {
        logger->error() << "Must specify input filename";
        return 1;
    }

    cv::Mat input = cv::imread(sInput);
    if(input.empty())
    {
        std::cerr << "Unable to read input file " << sInput << std::endl;
        return 1;
    }
    
    cv::resize(input, input, {width, input.rows * width/input.cols}, cv::INTER_CUBIC);
    
    // ########## FACE MESH LANDMARKER #########
    if(sModel.empty())
    {
        logger->error() << "Must specify 3D model";
        return 1;
    }
    
    if(sMapping.empty())
    {
        logger->error() << "Must specify landmark mapping";
        return 1;
    }
    
    drishti::face::FaceLandmarkMeshMapper mapper(sModel, sMapping);

    // ######### LANDMARK ######################
    if(sDetector.empty())
    {
        logger->error() << "Must specify detector file";
        return 1;
    }
    
    if(sRegressor.empty())
    {
        logger->error() << "Must specify regressor file";
        return 1;
    }

    std::shared_ptr<FaceLandmarker> landmarker = std::make_shared<FaceLandmarker>(sRegressor, sDetector);

    std::vector<cv::Point2f> landmarks;
    if(landmarker)
    {
        cv::Mat gray;
        cv::extractChannel(input, gray, 1);
        landmarks = (*landmarker)(gray, {});
    }

    if(!sOutput.empty())
    {
        cv::Mat canvas = input.clone();
        landmarker->draw(canvas);
        cv::imwrite(sOutput, canvas);
    }
    
    if(!sMapping.empty() && !sModel.empty() && landmarks.size())
    {
        cv::Mat iso;
        eos::render::Mesh mesh;
        cv::Point3f R = mapper(landmarks, input, mesh, iso);
        
        std::cout << "R = " << R << std::endl;
        
        // (((( Draw mesh for visualization ))))
        if(verbose)
        {
            for(auto & p : mesh.texcoords)
            {
                p[0] *= iso.cols;
                p[1] *= iso.rows;
            }
            
            for(int i = 0; i < mesh.tvi.size(); i++)
            {
                const auto &t = mesh.tvi[i];
                cv::Point2f v0 = mesh.texcoords[t[0]];
                cv::Point2f v1 = mesh.texcoords[t[1]];
                cv::Point2f v2 = mesh.texcoords[t[2]];
                cv::line(iso, v0, v1, {0,255,0}, 1, 8);
                cv::line(iso, v1, v2, {0,255,0}, 1, 8);
                cv::line(iso, v2, v0, {0,255,0}, 1, 8);
            }
            cv::imshow("iso", iso);
            cv::waitKey(0);
        }
    }
}
