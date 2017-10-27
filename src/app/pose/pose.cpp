/*! -*-c++-*-
 @file   pose.cpp
 @author David Hirvonen
 @brief  EOS deformable face mesh fitting test w/ rendering.
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/core/drishti_stdlib_string.h" // std::to_string() for android
#include "drishti/face/FaceMeshMapperEOSLandmark.h"
#include "drishti/face/FaceMeshMapperEOSLandmarkContour.h" // exp
#include "drishti/face/FaceMesh.h"
#include "drishti/face/FaceMeshMapperFactory.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimatorDEST.h"
#include "drishti/ml/ObjectDetectorCV.h"
#include "drishti/face/FaceDetector.h"
#include "drishti/core/Logger.h"
#include "drishti/core/string_utils.h"
#include "drishti/core/make_unique.h"
#include "drishti/geometry/motion.h"

#include "drishti/graphics/mesh.h"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif

#if defined(DRISHTI_DO_GPU_TESTING)
#  include "aglet/GLContext.h"
#  include "ogles_gpgpu/common/proc/gain.h"
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

#include <glm/glm.hpp>
#include <glm/gtc/matrix_transform.hpp>

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
static cv::Mat getImage(ogles_gpgpu::ProcInterface& proc, cv::Mat& frame);

struct Cursor
{
    using TimePoint = std::chrono::time_point<std::chrono::high_resolution_clock>;

    Cursor() {}
    Cursor(const cv::Point2f& position)
        : position(position)
    {
        valid = true;
        time = std::chrono::high_resolution_clock::now();
    }

    bool valid = false;
    cv::Point2f position;
    std::chrono::time_point<std::chrono::high_resolution_clock> time;
};

static glm::mat4 cvToGLM(const cv::Matx44f &MVP)
{
    glm::mat4 mvp;
    for(int y = 0; y < 4; y++)
    {
        for(int x = 0; x < 4; x++)
        {
            mvp[y][x] = MVP(y,x);
        }
    }
    
    return mvp;
}

int gauze_main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    auto logger = drishti::core::Logger::create("eos-pose");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sRegressor;
    std::string sDetector;
    std::string sFaceMeshMapperFactory;
    std::string sBoilerplate;
    std::string sInput;
    std::string sOutput;
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
        ("width", "Width", cxxopts::value<int>(width))("v,verbose", "verbose", cxxopts::value<bool>(verbose))

        ("preview", "preview", cxxopts::value<bool>(doPreview))
        ("cursor", "Demo rendering mode", cxxopts::value<bool>(doCursor))
        ("wire", "Do wireframe rendering", cxxopts::value<bool>(doWire))
        ("clear", "Clear background prior to rendering", cxxopts::value<bool>(doClear))
    
        ("d,detector", "detector", cxxopts::value<std::string>(sDetector))
        ("r,regressor", "regressor", cxxopts::value<std::string>(sRegressor))
        ("factory", "FaceMeshMapperFactory json file", cxxopts::value<std::string>(sFaceMeshMapperFactory))
        ("boilerplate", "FaceMeshMapperFactory boilerplate", cxxopts::value<std::string>(sBoilerplate))

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
    
    if(!sBoilerplate.empty())
    {
        drishti::face::FaceMeshMapperFactory::serialize(sBoilerplate);
        return 0;
    }

    // ### Directory
    if (sOutput.empty())
    {
        logger->error("Must specify output directory");
        return 1;
    }

    if (sInput.empty())
    {
        logger->error("Must specify input filename");
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
    if (sFaceMeshMapperFactory.empty())
    {
        logger->error("Must specify a valid mesh factory input file");
        return 1;
    }
    auto mapper = drishti::face::FaceMeshMapperFactory(sFaceMeshMapperFactory).create();

    // ######### DETECTOR ######################
    if (sDetector.empty())
    {
        logger->error("Must specify detector file");
        return 1;
    }
    auto detector = std::make_shared<drishti::ml::ObjectDetectorCV>(sDetector);
    detector->setMinNeighbors(3);

    // ######### LANDMARKS ######################
    if (sRegressor.empty())
    {
        logger->error("Must specify regressor file");
        return 1;
    }
    auto landmarker = std::make_shared<drishti::ml::RegressionTreeEnsembleShapeEstimatorDEST>(sRegressor);
    
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

        {
            auto result = (*mapper)(landmarks, input);
            cv::Mat iso = result->extractTexture(input);
            cv::Point3f Reuler = result->getRotation();

            logger->info("rotation: {} {} {}", Reuler.x, Reuler.y, Reuler.z);

            // (((( Draw mesh for visualization ))))
            if (doPreview || !sOutput.empty())
            {
#if defined(DRISHTI_DO_GPU_TESTING)
                // Create a texture from teh isomap
                cv::Mat frame;
                cv::cvtColor(input, frame, cv::COLOR_BGR2BGRA);

                // Create a window/context:
                const auto kType = aglet::GLContext::kAuto;
                auto opengl = aglet::GLContext::create(kType, doPreview ? "eos" : "", input.cols, input.rows);
                opengl->resize(frame.cols, frame.rows);
                (*opengl)();
                
                drishti::graphics::MeshTex mesh;
                result->getFaceMesh(mesh);

                // Perform texture swizzling:
                ogles_gpgpu::VideoSource source;
                ogles_gpgpu::MeshProc warper(mesh, iso, doWire);
                source.set(&warper);

                if (doClear)
                {
                    warper.setBackground({ 0.f, 0.f, 0.f, 1.f });
                }

                // Set shader model view projection:
                const auto frustum = result->getFrustum();
                const auto projection = glm::ortho<float>(frustum.l, frustum.r, frustum.t, frustum.b, -100.0, 100.0);
                {
                    glm::mat4 mvp = cvToGLM(result->getModelViewProjection());
                    const auto modelViewProj = projection * mvp;
                    warper.setModelViewProjection(modelViewProj);
                }

                std::shared_ptr<ogles_gpgpu::Disp> display;
                if (doPreview && opengl->hasDisplay())
                {
                    display = std::make_shared<ogles_gpgpu::Disp>();
                    display->setOutputRenderOrientation(ogles_gpgpu::RenderOrientationFlipped);
                    warper.add(display.get());
                }

                auto R = result->getRotation(); // get rotation
  
                Cursor cursor[2]; // keep cursor in scope
                if (doCursor)
                {
                    std::function<void(double x, double y)> cursorCallback = [&](double x, double y)
                    {
                        if (!cursor[0].valid)
                        {
                            cursor[0] = Cursor(cv::Point2f(x, y));
                            return;
                        }

                        cursor[1] = Cursor(cv::Point2f(x, y));
                        const auto delta = cursor[1].position - cursor[0].position;
                        const double elapsed = std::chrono::duration<double>(cursor[1].time - cursor[0].time).count();
                        static const float sensitivity = elapsed * 64.f;
                        static const float xRange = static_cast<float>(180.f) / 6.0f;
                        static const float yRange = static_cast<float>(180.f) / 6.0f;
                        
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
                    result->setRotation(R);
                    const auto modelViewProj = projection * cvToGLM(result->getModelViewProjection());
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

                if (display && doPreview)
                {
                    opengl->setWait(!doCursor);
                    (*opengl)(render);
                }
                else
                {
                    render(); // call it once and quit
                }
#endif // defined(DRISHTI_DO_GPU_TESTING)

                if (!sOutput.empty())
                {
#if defined(DRISHTI_DO_GPU_TESTING)
                    // Capture last rendered image:
                    cv::Mat rendered;
                    getImage(warper, rendered);
                    cv::imwrite(sOutput + "/" + sBase + "_render.png", rendered);
#endif // DRISHTI_DO_GPU_TESTING
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

static cv::Mat getImage(ogles_gpgpu::ProcInterface& proc, cv::Mat& frame)
{
    if (dynamic_cast<ogles_gpgpu::MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        ogles_gpgpu::MemTransfer::FrameDelegate delegate = [&](const ogles_gpgpu::Size2d& size, const void* pixels, size_t bytesPerRow) {
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
