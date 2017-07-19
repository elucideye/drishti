/*!
 @file   pose.cpp
 @author David Hirvonen
 @brief  EOS deformable face mesh fitting test w/ rendering.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/face/FaceMeshMapperLandmark.h"
#include "drishti/face/FaceMeshMapperLandmarkContour.h" // exp
#include "drishti/face/FaceMesh.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimatorDEST.h"
#include "drishti/ml/ObjectDetectorCV.h"
#include "drishti/core/Logger.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/make_unique.h"
#include "drishti/geometry/motion.h"

#include "drishti/graphics/mesh.h"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif

#if defined(DRISHTI_HAS_GLTEST)
#  include "aglet/GLContext.h"
#  include "drishti/graphics/gain.h"
#  include "ogles_gpgpu/common/proc/disp.h"
#  include "ogles_gpgpu/common/proc/video.h"
#  include "ogles_gpgpu/common/gl/memtransfer_optimized.h"
#  ifdef ANDROID
#    define TEXTURE_FORMAT GL_RGBA
#  else
#    define TEXTURE_FORMAT GL_BGRA
#  endif
#endif
// clang-format on

#include <opencv2/imgproc.hpp>
#include <opencv2/highgui.hpp>

#include "cxxopts.hpp"

#include <iostream>
#include <sstream>
#include <numeric>

using Landmarks = std::vector<cv::Point2f>;

static void* void_ptr(const cv::Mat& image)
{
    return const_cast<void*>(image.ptr<void>());
}

static std::vector<float> parseVec3f(const std::string& R);
static cv::Mat getImage(ogles_gpgpu::ProcInterface &proc, cv::Mat &frame);

struct Cursor
{
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;
    
    Cursor() {}
    Cursor(const cv::Point2f &position) : position(position)
    {
        valid = true;
        time = std::chrono::high_resolution_clock::now();
    }
    
    bool valid = false;
    cv::Point2f position;
    std::chrono::time_point<std::chrono::high_resolution_clock> time;
};

int gauze_main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    auto logger = drishti::core::Logger::create("eos-pose");

    // ############################
    // ### Command line parsing ###
    // ############################

    drishti::face::FaceMeshMapperLandmarkContour::Assets assets;

    std::string sRegressor;
    std::string sDetector;
    std::string sTriangles;
    std::string sInput;
    std::string sOutput;
    std::string sRotation;
    bool verbose = false;
    bool doPreview = false;
    bool doWire = false;
    bool doClear = false;
    bool doCursor = false;
    int threads = -1;
    int width = 256;
    
    cxxopts::Options options("eos-pose", "Command line interface for eos 2d->3d face model fitting");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("w,width", "Width", cxxopts::value<int>(width))("v,verbose", "verbose", cxxopts::value<bool>(verbose))

        ("p,preview", "preview", cxxopts::value<bool>(doPreview))
        ("cursor", "Demo rendering mode", cxxopts::value<bool>(doCursor))
        ("wire", "Do wireframe rendering", cxxopts::value<bool>(doWire))
        ("clear", "Clear background prior to rendering", cxxopts::value<bool>(doClear))
        ("rotation", "Rotation: x,y,z", cxxopts::value<std::string>(sRotation))
    
        ("d,detector", "detector", cxxopts::value<std::string>(sDetector))
        ("r,regressor", "regressor", cxxopts::value<std::string>(sRegressor))
        ("3,triangles", "delaunay triangles", cxxopts::value<std::string>(sTriangles))
    
        ("model", "3D dephormable model", cxxopts::value<std::string>(assets.model))
        ("mapping", "Landmark mapping", cxxopts::value<std::string>(assets.mappings))
        ("model-contour", "Model contour indices", cxxopts::value<std::string>(assets.contour))
        ("edge-topology", "Model's precomputed edge topology", cxxopts::value<std::string>(assets.edgetopology))
        ("blendshapes", "Blendshapes", cxxopts::value<std::string>(assets.blendshapes))
    
        ("t,threads", "Thread count", cxxopts::value<int>(threads))
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
    if (assets.model.empty())
    {
        logger->error() << "Must specify 3D model";
        return 1;
    }

    if (assets.mappings.empty())
    {
        logger->error() << "Must specify landmark mapping";
        return 1;
    }
    
    std::unique_ptr<drishti::face::FaceMeshMapper> mapper =
        drishti::core::make_unique<drishti::face::FaceMeshMapperLandmarkContour>(assets);

    // One can instantiate a simple landmark mapper for faster performance:
    // drishti::core::make_unique<drishti::face::FaceMeshMapperLandmark>(assets.model, assets.mappings);

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
                logger->info("No faces found");
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

        std::string sBase = drishti::core::basename(sInput);

        if (!assets.mappings.empty() && !assets.model.empty() && landmarks.size())
        {
            auto result = (*mapper)(landmarks, input);
            cv::Mat iso = drishti::face::extractTexture(result, input);
            cv::Point3f Reuler = drishti::face::getRotation(result.rendering_params);
            
            logger->info("rotation: {}", Reuler);
            
            // (((( Draw mesh for visualization ))))
            if (doPreview || !sOutput.empty())
            {
#if defined(DRISHTI_HAS_GLTEST)
                // Create a texture from teh isomap
                cv::Mat frame;
                cv::cvtColor(input, frame, cv::COLOR_BGR2BGRA);
                
                // Create a window/context:
                const auto kType = aglet::GLContext::kAuto;
                auto opengl = aglet::GLContext::create(kType, doPreview ? "eos" : "", input.cols, input.rows);
                opengl->resize(frame.cols, frame.rows);
                (*opengl)();
                
                // Create triangles from mesh:
                ogles_gpgpu::Mesh mesh { result.mesh.vertices, result.mesh.texcoords, result.mesh.tvi };

                // Perform texture swizzling:
                ogles_gpgpu::VideoSource source;
                ogles_gpgpu::MeshProc warper(mesh, iso, doWire);
                source.set(&warper);
                
                if(doClear)
                {
                    warper.setBackground({0.f,0.f,0.f,1.f});
                }
                
                // Set shader model view projection:
                const auto frustum = result.rendering_params.get_frustum();
                const auto projection = glm::ortho<float>(frustum.l, frustum.r, frustum.t, frustum.b, -100.0, 100.0);
                {
                    const auto modelViewProj = projection * result.rendering_params.get_modelview();
                    warper.setModelViewProjection(modelViewProj);
                }
                
                std::shared_ptr<ogles_gpgpu::Disp> display;
                if (doPreview && opengl->hasDisplay())
                {
                    display = std::make_shared<ogles_gpgpu::Disp>();
                    display->setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
                    warper.add(display.get());
                }
                
                auto R = result.rendering_params.get_rotation();
                if (!sRotation.empty())
                {
                    auto rotations = parseVec3f(sRotation);
                    R.x = rotations[0];
                    R.y = rotations[1];
                    R.z = rotations[2];
                }
                
                Cursor cursor[2]; // keep cursor in scope
                if(doCursor)
                {
                    std::function<void(double x, double y)> cursorCallback = [&](double x, double y)
                    {
                        if(!cursor[0].valid)
                        {
                            cursor[0] = Cursor(cv::Point2f(x, y));
                            return;
                        }
                        
                        cursor[1] = Cursor( cv::Point2f(x, y) );
                        const auto delta = cursor[1].position - cursor[0].position;
                        const double elapsed = std::chrono::duration<double>(cursor[1].time - cursor[0].time).count();
                        static const float sensitivity = elapsed / 10.f;
                        static const float xRange = static_cast<float>(M_PI)/8.0f;
                        static const float yRange = static_cast<float>(M_PI)/8.0f;
                        R.y = std::max(std::min(R.y + delta.x * sensitivity, +yRange), -xRange);
                        R.x = std::max(std::min(R.x + delta.y * sensitivity, +yRange), -yRange);
                        std::swap(cursor[1], cursor[0]);
                        
                        logger->info("X = {} Y = {}", x, y);
                    };
                    opengl->setCursorCallback(cursorCallback);
                }
                
                std::function<bool(void)> render = [&]()
                {
                    // Alway update transformation:
                    result.rendering_params.set_rotation(glm::quat(R));
                    const auto modelViewProj = projection * result.rendering_params.get_modelview();
                    warper.setModelViewProjection(modelViewProj);
                    
                    source({ { frame.cols, frame.rows }, void_ptr(frame), true, 0, TEXTURE_FORMAT });
                    
                    if (display)
                    {
                        auto& geometry = opengl->getGeometry();
                        display->setOffset(geometry.tx, geometry.ty);
                        display->setDisplayResolution(geometry.sx, geometry.sy);
                    }
                    return true;
                };
                
                if(display && doPreview)
                {
                    opengl->setWait(!doCursor);
                    (*opengl)(render);
                }
                else
                {
                    render(); // call it once and quit
                }
#endif // defined(DRISHTI_HAS_GLTEST)

                if (!sOutput.empty())
                {
#if defined(DRISHTI_HAS_GLTEST)                    
                    // Capture last rendered image:
                    cv::Mat rendered;
                    getImage(warper, rendered);
                    cv::imwrite(sOutput + "/" + sBase + "_render.png", rendered);
#endif // DRISHTI_HAS_GLTEST
                    cv::imwrite(sOutput + "/" + sBase + "_iso.png", iso);
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
    catch (const std::exception& e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
    }
}

static cv::Mat getImage(ogles_gpgpu::ProcInterface &proc, cv::Mat &frame)
{
    if(dynamic_cast<ogles_gpgpu::MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        ogles_gpgpu::MemTransfer::FrameDelegate delegate = [&](const ogles_gpgpu::Size2d &size, const void *pixels, size_t bytesPerRow)
        {
            frame = cv::Mat(size.height, size.width, CV_8UC4, (void*)pixels, bytesPerRow).clone();
        };
        proc.getResultData(delegate);
    }
    else
    {
        frame.create(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4); // noop if preallocated
        proc.getResultData(frame.ptr());
    }
    return frame;
}

// #############################################
// ### BNF parser:                           ###
// ### vector<float> = {1.0,2.0,3.0}         ###
// #############################################

#include <boost/spirit/include/qi.hpp>

namespace qi = boost::spirit::qi;

template <typename Iterator, typename Skipper = qi::blank_type>
struct vec3f_parser : qi::grammar<Iterator, std::vector<float>(), Skipper>
{
    vec3f_parser()
    : vec3f_parser::base_type(start)
    {
        start = qi::repeat(2)[(qi::float_ >> ',')] >> qi::float_;
    }
    qi::rule<Iterator, std::vector<float>(), Skipper> start;
};

template <typename Iterator>
std::vector<float> parseVec3f(Iterator first, Iterator last)
{
    std::vector<float> v;
    vec3f_parser<decltype(first)> parser;
    bool result = qi::phrase_parse(first, last, parser, qi::blank, v);
    return v;
}

static std::vector<float> parseVec3f(const std::string& R)
{
    return parseVec3f(R.begin(), R.end());
}

