/*!
 @file   drishti/hci/FaceFinder.cpp
 @author David Hirvonen
 @brief  Face detection and tracking class with GPU acceleration.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/hci/FaceFinder.h"
#include "drishti/hci/gpu/FacePainter.h"
#include "drishti/hci/gpu/FlashFilter.h"

#include "drishti/face/gpu/EyeFilter.h"
#include "drishti/eye/gpu/EllipsoPolarWarp.h"
#include "drishti/eye/gpu/EyeWarp.h"
#include "drishti/core/Logger.h"
#include "drishti/core/timing.h"
#include "drishti/core/scope_guard.h"
#include "drishti/core/drishti_operators.h"
#include "drishti/core/scope_guard.h"
#include "drishti/core/make_unique.h"
#include "drishti/face/FaceDetectorAndTracker.h"
#include "drishti/face/FaceModelEstimator.h"
#include "drishti/face/FaceDetector.h"
#include "drishti/geometry/motion.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/hci/EyeBlob.h"

#include "ogles_gpgpu/common/proc/fifo.h"
#include "drishti/graphics/swizzle.h"

#include <vector>
#include <memory>
#include <chrono>

// TODO: centralize this (static method?)
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#  define TEXTURE_FORMAT_IS_RGBA 1
#else
#  define TEXTURE_FORMAT GL_BGRA
#  define TEXTURE_FORMAT_IS_RGBA 0
#endif

#define DRISHTI_HCI_FACEFINDER_LANDMARKS_WIDTH 1024

#define DRISHTI_HCI_FACEFINDER_FLOW_WIDTH 256
#define DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER 0 // *** display ***
#define DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT 1 // *** display ***

#define DRISHTI_HCI_FACEFINDER_FLASH_WIDTH 128

#define DRISHTI_HCI_FACEFINDER_DETECTION_WIDTH 512
#define DRISHTI_HCI_FACEFINDER_DO_TRACKING 1
#define DRISHTI_HCI_FACEFINDER_DO_ACF_MODIFY 1

#define DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES 1
#define DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES_DISPLAY 1

#define DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS 0
#define DRISHTI_HCI_FACEFINDER_LOG_DETECTIONS 0

#define DRISHTI_HCI_FACEFINDER_ENABLE_ACF_FILTER_LOGGING 0

static const char * sBar = "#################################################################";

// === utility ===

using drishti::face::operator*;
using drishti::core::operator*;

DRISHTI_HCI_NAMESPACE_BEGIN

static int getDetectionImageWidth(float, float, float, float, float);

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT
static cv::Size uprightSize(const cv::Size &size, int orientation);
static void extractFlow(const cv::Mat4b &ayxb, const cv::Size &frameSize, ScenePrimitives &scene, float flowScale = 1.f);
#endif

#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS
static cv::Mat draw(const drishti::acf::Detector::Pyramid &pyramid);
static void logPyramid(const std::string &filename, const drishti::acf::Detector::Pyramid &P);
#endif // DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

static ogles_gpgpu::Size2d convert(const cv::Size &size)
{
    return ogles_gpgpu::Size2d(size.width, size.height);
}

FaceFinder::FaceFinder(std::shared_ptr<drishti::face::FaceDetectorFactory> &factory, Settings &args, void *glContext)
: m_glContext(glContext)
, m_hasInit(false)
, m_outputOrientation(args.outputOrientation)

// ######## DO Landmarks #########
, m_doLandmarks(args.doLandmarks)
, m_landmarksWidth(DRISHTI_HCI_FACEFINDER_LANDMARKS_WIDTH)

// ######## DO FLOW #########
, m_doFlow(args.doFlow)
, m_flowWidth(DRISHTI_HCI_FACEFINDER_FLOW_WIDTH)

// ###### DO FLASH ##########
, m_doFlash(args.doFlash)
, m_flashWidth(DRISHTI_HCI_FACEFINDER_FLASH_WIDTH)

, m_doIris(true)//DRISHTI_HCI_FACEFINDER_DO_ELLIPSO_POLAR)

, m_factory(factory)
, m_sensor(args.sensor)
, m_logger(args.logger)
, m_threads(args.threads)
, m_minDistanceMeters(args.minDetectionDistance)
, m_maxDistanceMeters(args.maxDetectionDistance)
{
    m_debugACF = false;
    m_doFlash = true;
    
    m_doDifferenceEyes = DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES;
    m_doDifferenceEyesDisplay = DRISHTI_HCI_FACEFINDER_DO_DIFFERENCE_EYES_DISPLAY;
}

std::unique_ptr<FaceFinder> FaceFinder::create(FaceDetectorFactoryPtr &factory, Settings &settings, void *glContext)
{
    auto finder = drishti::core::make_unique<FaceFinder>(factory, settings, glContext);
    finder->initialize();
    return finder;
}

// Initialize outside of constructor for virtual calls:
void FaceFinder::initialize()
{
    m_hasInit = true;
    init2(*m_factory);
    init(m_sensor->intrinsic().getSize());
}

FaceFinder::~FaceFinder()
{
    std::unique_lock<std::mutex> lock(m_mutex);
}

bool FaceFinder::needsDetection(const TimePoint &now) const
{
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - m_objects.first).count();
    return (elapsed > m_faceFinderInterval);
}

float FaceFinder::getMinDistance() const
{
    return m_minDistanceMeters;
}

float FaceFinder::getMaxDistance() const
{
    return m_maxDistanceMeters;
}

void FaceFinder::setDoCpuAcf(bool flag)
{
    m_doCpuACF = flag;
}

bool FaceFinder::getDoCpuAcf() const
{
    return m_doCpuACF;
}

void FaceFinder::setFaceFinderInterval(double interval)
{
    m_faceFinderInterval = interval;
}

double FaceFinder::getFaceFinderInterval() const
{
    return m_faceFinderInterval;
}

void FaceFinder::registerFaceMonitorCallback(FaceMonitor *callback)
{
    m_faceMonitorCallback.push_back(callback);
}

void FaceFinder::dumpEyes(std::vector<cv::Mat4b> &frames, std::vector<std::array<eye::EyeModel,2>> &eyes)
{
    m_eyeFilter->dump(frames, eyes);
}

void FaceFinder::dumpFaces(std::vector<cv::Mat4b> &frames)
{
    if(m_fifo->getBufferCount() == m_fifo->getProcPasses().size())
    {
        frames.resize(m_fifo->getBufferCount());
        for(int i = 0; i < frames.size(); i++)
        {
            auto *filter = (*m_fifo)[i];

            cv::Size outSize(filter->getOutFrameW(), filter->getOutFrameH());
            frames[i].create(outSize.height, outSize.width);
            filter->getResultData(frames[i].ptr<uint8_t>());
        }
    }
}

int FaceFinder::computeDetectionWidth(const cv::Size &inputSizeUp) const
{
    // TODO: Add global constant and set limits on reasonable detection size 
    const float faceWidthMeters = 0.120;
    const float fx = m_sensor->intrinsic().m_fx;
    const float winSize = m_detector->getWindowSize().width;
    return getDetectionImageWidth(faceWidthMeters, fx, m_maxDistanceMeters, winSize, inputSizeUp.width);
}

// Side effect: set m_pyramdSizes
void FaceFinder::initACF(const cv::Size &inputSizeUp)
{
    // ### ACF (Transpose) ###
    // Find the detection image width required for object detection at the max distance:
    int detectionWidth = computeDetectionWidth(inputSizeUp);
    m_ACFScale = float(inputSizeUp.width) / float(detectionWidth);
    
    // ACF implementation uses reduce resolution transposed image:
    cv::Size detectionSize = inputSizeUp * (1.0f/m_ACFScale);
    cv::Mat I(detectionSize.width, detectionSize.height, CV_32FC3, cv::Scalar::all(0));
    MatP Ip(I);
    m_detector->computePyramid(Ip, m_P);
    
    m_pyramidSizes.resize(m_P.nScales);
    std::vector<ogles_gpgpu::Size2d> sizes(m_P.nScales);
    for(int i = 0; i < m_P.nScales; i++)
    {
        const auto size = m_P.data[i][0][0].size();
        sizes[i] = { size.width * 4, size.height * 4 }; // undo ACF binning x4
        m_pyramidSizes[i] = { size.width * 4, size.height * 4 };
        
        // CPU processing works with tranposed images for col-major storage assumption.
        // Undo that here to map to row major representation.  Perform this step
        // to make transpose operation explicit.
        std::swap(sizes[i].width, sizes[i].height);
        std::swap(m_pyramidSizes[i].width, m_pyramidSizes[i].height);
    }
    
    const int grayWidth = m_doLandmarks ? m_landmarksWidth : 0;
    const int flowWidth = m_doFlow ? m_flowWidth : 0;
    const bool do10Channel = true;
    const auto featureKind = do10Channel ? ogles_gpgpu::ACF::kLUVM012345 : ogles_gpgpu::ACF::kLM012345;
    const ogles_gpgpu::Size2d size(inputSizeUp.width,inputSizeUp.height);
    m_acf = std::make_shared<ogles_gpgpu::ACF>(m_glContext, size, sizes, featureKind, grayWidth, flowWidth, m_debugACF);
    m_acf->setRotation(m_outputOrientation);
    m_acf->setLogger(m_logger);
}

// ### Fifo ###
void FaceFinder::initFIFO(const cv::Size &inputSize, std::size_t n)
{
    m_fifo = std::make_shared<ogles_gpgpu::FifoProc>(n);
    m_fifo->init(inputSize.width, inputSize.height, INT_MAX, false);
    m_fifo->createFBOTex(false);
}

void FaceFinder::initFlasher()
{
    // ### Flash ###
    if(m_doDifferenceEyes)
    {
        assert(m_eyeFilter.get());
        m_flasher = std::make_shared<ogles_gpgpu::FlashFilter>(ogles_gpgpu::FlashFilter::kCenteredDifference);
        m_flasher->init(128, 64, INT_MAX, false); // smoothProc.setOutputSize(48, 0);
        m_flasher->createFBOTex(false);
        
        if(m_doDifferenceEyesDisplay)
        {
            m_eyeFilter->getOutputFilter()->add(m_flasher->getInputFilter());
        }
        else
        {
            m_eyeFilter->getInputFilter()->add(m_flasher->getInputFilter());
        }
    }
    else
    {
        assert(m_acf.get());
        m_flasher = std::make_shared<ogles_gpgpu::FlashFilter>(ogles_gpgpu::FlashFilter::kLaplacian);
        m_flasher->init(48, 48, INT_MAX, false); // smoothProc.setOutputSize(48, 0);
        m_flasher->createFBOTex(false);
        m_acf->getRgbSmoothProc()->add(m_flasher->getInputFilter());
    }
}

void FaceFinder::initIris(const cv::Size &size)
{
    // ### Ellipsopolar warper ####
    for(int i = 0; i < 2; i++)
    {
        m_ellipsoPolar[i] = std::make_shared<ogles_gpgpu::EllipsoPolarWarp>();
        m_ellipsoPolar[i]->setOutputSize(size.width, size.height);
    }
}

void FaceFinder::initEyeEnhancer(const cv::Size &inputSizeUp, const cv::Size &eyesSize)
{
    // ### Eye enhancer ###
    const auto mode = ogles_gpgpu::EyeFilter::kMean3;
    const float cutoff = 0.5;
    
    m_eyeFilter = std::make_shared<ogles_gpgpu::EyeFilter>(convert(eyesSize), mode, cutoff);
    m_eyeFilter->setAutoScaling(true);
    m_eyeFilter->setOutputSize(eyesSize.width, eyesSize.height);
    
    if(m_doIris)
    {
        // Add a callback to retrieve updated eye models automatically:
        cv::Matx33f N = transformation::scale(0.5, 0.5) * transformation::translate(1.f, 1.f);
        for(int i = 0; i < 2; i++)
        {
            std::function<drishti::eye::EyeWarp()> eyeDelegate = [&, N, i]()
            {
                auto eye = m_eyeFilter->getEyeWarps()[i];
                eye.eye = N * eye.H * eye.eye;
                return eye;
            };
            m_ellipsoPolar[i]->addEyeDelegate(eyeDelegate);
            m_eyeFilter->add(m_ellipsoPolar[i].get());
        }
    }
    
    if(m_doEyeFlow)
    { // optical flow for eyes:
        m_eyeFlow = std::make_shared<ogles_gpgpu::FlowOptPipeline>(0.004, 1.0, false);
#if TEXTURE_FORMAT_IS_RGBA
        m_eyeFlowBgra = std::make_shared<ogles_gpgpu::SwizzleProc>();
        m_eyeFlow->add(m_eyeFlowBgra.get());
        m_eyeFlowBgraInterface = m_eyeFlowBgra.get();
#else
        m_eyeFlowBgraInterface = m_eyeFlow.get();
#endif
        
        m_eyeFilter->add(m_eyeFlow.get());
    }
    
    m_eyeFilter->prepare(inputSizeUp.width, inputSizeUp.height, (GLenum)GL_RGBA);
}

void FaceFinder::initPainter(const cv::Size & /* inputSizeUp */ )
{
    m_logger->info() << "Init painter";
}

void FaceFinder::init(const cv::Size &inputSize)
{
    //m_logger->info() << "FaceFinder::init()";
    //m_logger->set_level(spdlog::level::err);

    m_start = HighResolutionClock::now();
    
    auto inputSizeUp = inputSize;
    bool hasTranspose = ((m_outputOrientation / 90) % 2);
    if(hasTranspose)
    {
        std::swap(inputSizeUp.width, inputSizeUp.height);
    }
    
    m_faceEstimator = std::make_shared<drishti::face::FaceModelEstimator>(*m_sensor);
    
    initColormap();
    initFIFO(inputSizeUp, 3); // keep last 3 frames
    initACF(inputSizeUp);
    initPainter(inputSizeUp); // {inputSizeUp.width/4, inputSizeUp.height/4}
    
    if(m_doIris)
    {
        initIris({640, 240});
    }
    
    // Must initial eye filter before flasher:
    initEyeEnhancer(inputSizeUp, m_eyesSize);
    
    if(m_doFlash)
    {
        // Must initialize flasher after eye filter:
        initFlasher();
    }
}

// ogles_gpgpu::VideoSource can support list of subscribers
//
//       +=> FIFO[1][0] == ACF ====>
// VIDEO |       |
//       +=======+======== FLOW ===>

// Illustrate circular FIFO for size == 2 and 3
//
//   (2 frame)   (3 frame)
// 0 : [0][ ]	 [0][ ][ ]
//     [I][ ]	 [I][ ][ ]
//     [O][ ]	 [O][ ][ ]
//
// 1 : [0][1]	 [0][1][ ]
//     [ ][I]	 [ ][I][ ]
//     [O][ ]	 [O][ ][ ]
//
// 2 : [2][1]	 [0][1][2]
//     [I][ ]	 [ ][ ][I]
//     [ ][O]	 [O][ ][ ]
//
// 3 : [2][3]	 [3][1][2]
//     [ ][I]	 [I][ ][ ]
//     [O][ ]	 [ ][O][ ]
//
// 4 : [4][3]	 [3][4][2]
//     [I][ ]	 [ ][I][ ]
//     [ ][O]	 [ ][ ][O]
//
// 5 : [4][5]	 [3][4][5]
//     [ ][I]	 [O][ ][ ]
//     [O][ ]	 [ ][ ][I]

GLuint FaceFinder::operator()(const FrameInput &frame1)
{
    assert(m_hasInit);
    assert(m_sensor->intrinsic().getSize() == cv::Size(frame1.size.width, frame1.size.height));
    
    std::string methodName = DRISHTI_LOCATION_SIMPLE;
    core::ScopeTimeLogger faceFinderTimeLogger = [this, methodName](double elapsed)
    {
        if(m_logger)
        {
            m_logger->info() << "TIMING:" << methodName << " : " << m_timerInfo << " full=" << elapsed;
        }
    };
    
    // Get current timestamp
    const auto &now = faceFinderTimeLogger.getTime();
    
    const bool doDetection = needsDetection(now);

    m_frameIndex++; // increment frame index
    
    // Run GPU based processing on current thread and package results as a task for CPU
    // processing so that it will be available on the next frame.  This method will compute
    // ACF output using shaders on the GPU, and may optionally extract other GPU related
    // features.
    ScenePrimitives scene1(m_frameIndex), scene0, *outputScene = nullptr; // time: n+1 and n
    {
        core::ScopeTimeLogger preprocessTimeLogger = [this](double t) { m_timerInfo.acfProcessingTime = t; };
        preprocess(frame1, scene1, doDetection);
    }
    
    // Initialize input texture with ACF upright texture:
    GLuint texture1 = m_acf->first()->getOutputTexId(), texture0 = 0, outputTexture = 0;
    
    if(m_threads)
    {
        // Retrieve the previous frame and scene
        if((m_fifo->getBufferCount() > 0) && doAnnotations())
        {
            {
                // Retrieve the previous frame (latency == 1)
                // from our N frame FIFO.
                //core::ScopeTimeLogger paintTimeLogger = [this](double t) { m_logger->info() << "WAITING: " << t; };
                scene0 = m_scene.get(); // scene n-1
                texture0 = (*m_fifo)[-1]->getOutputTexId(); // texture n-1
            }
            
            updateEyes(texture0, scene0); // update the eye texture
            
            { // Explicit output variable configuration:
                core::ScopeTimeLogger glTimeLogger = [this](double t) { m_timerInfo.renderSceneTimeLogger(t); };
                outputTexture = paint(scene0, texture0);
                outputScene = &scene0;
            }
        }
        else
        {
            outputTexture = texture1;
            outputScene = &scene0; // empty
        }

        // Eenque the current frame and scene for CPU processing so
        // that results will be available for the next step (see above).
        m_scene = m_threads->process([scene1,frame1,doDetection,this]() {
            ScenePrimitives sceneOut = scene1;
            detect(frame1, sceneOut, doDetection);
            if(doAnnotations())
            {
                // prepare line drawings for rendering while gpu is busy
                sceneOut.draw();
                
                // TODO: Eye contours for current scene can be created
                // for next frame here on the CPU thread.                
            }
            return sceneOut;
        });
    }
    else
    {
        detect(frame1, scene1, doDetection);
        
        if(doAnnotations())
        {
            updateEyes(texture1, scene1);
            
            // Excplicit output variable configuration:
            outputTexture = paint(scene1, texture1); // was 1
            outputScene = &scene1;
        }
        else
        {
            outputTexture = texture1;
            outputScene = &scene1; // empty
        }
    }
    
    // Add the current frame to FIFO
    m_fifo->useTexture(texture1, 1);
    m_fifo->render();
    
    // Clear face motion estimate, update window:
    m_faceMotion = { 0.f, 0.f, 0.f };
    m_scenePrimitives.push_front(*outputScene);

    if(m_scenePrimitives.size() >= 3)
    {
        m_scenePrimitives.pop_back();
    }
    
    if(m_scenePrimitives.size() >= 2)
    {
        if(m_scenePrimitives[0].faces().size() && m_scenePrimitives[1].faces().size())
        {
            const auto &face0 = m_scenePrimitives[0].faces()[0];
            const auto &face1 = m_scenePrimitives[1].faces()[0];
            const cv::Point3f delta = (*face0.eyesCenter - *face1.eyesCenter);
            m_faceMotion = delta; // active face motion
        }
    }
    
    try
    {
        this->notifyListeners(*outputScene, now, m_fifo->isFull());
    }
    catch(...) {}
    
    return outputTexture;
}

static bool hasValidFaceRequest(FaceMonitor &monitor, const ScenePrimitives &scene, const FaceMonitor::TimePoint &now)
{
    for(auto &face : scene.faces())
    {
        if(monitor.isValid((*face.eyesCenter), now))
        {
            return true;
        }
    }
    return false;
}

// Query list of listeners for valid face image
bool FaceFinder::hasValidFaceRequest(const ScenePrimitives &scene, const TimePoint &now) const
{
    for(auto &callback : m_faceMonitorCallback)
    {
        if(::drishti::hci::hasValidFaceRequest(*callback, scene, now))
        {
            return true;
        }
    }
    return false;
}

/**
 * Provide per frame scene descriptor callbacks.
 * Notably, if a face requet listener is registered,
 * then a response is expected on a per frame basis.
 */

void FaceFinder::notifyListeners(const ScenePrimitives &scene, const TimePoint &now, bool isInit)
{
    // Perform optional frame grabbing
    // NOTE: This must occur in the main OpenGL thread:
    std::vector<FaceMonitor::FaceImage> frames;
    
    // Build a list of active requests:
    bool hasActive = false;
    std::vector<bool> isActive(m_faceMonitorCallback.size(), false);

    if(scene.faces().size())
    {
        // 1) If any active face request is satisifed grab a frame+face buffer:
        for(int i = 0; i < m_faceMonitorCallback.size(); i++)
        {
            auto &callback = m_faceMonitorCallback[i];
            isActive[i] = ::drishti::hci::hasValidFaceRequest(*callback, scene, now);
            hasActive |= isActive[i];
        }

        if(hasActive)
        {
            // ### collect face images ###
            std::vector<cv::Mat4b> faces;
            dumpFaces(faces);

            if(faces.size())
            {
                frames.resize(faces.size());
                for(int i = 0; i < frames.size(); i++)
                {
                    frames[i].image = faces[i];
                }
                // Tag active face image with model
                frames[0].faceModels = scene.faces();
                
                // ### collect eye images ###
                std::vector<cv::Mat4b> eyes;
                std::vector<std::array<eye::EyeModel, 2>> eyePairs;
                dumpEyes(eyes, eyePairs);
                for(int i = 0; i < std::min(eyes.size(), faces.size()); i++)
                {
                    frames[i].eyes = eyes[i];
                }

                // ### Add the eye difference image ###
                cv::Mat4b filtered(m_eyeFilter->getOutFrameH(), m_eyeFilter->getOutFrameW());
                m_eyeFilter->getResultData(filtered.ptr());
                frames[0].extra = filtered;
            }
        }
    }
    
    // 3) Provide face images as requested:
    for(int i = 0; i < m_faceMonitorCallback.size(); i++)
    {
        if(isActive[i])
        {
            m_faceMonitorCallback[i]->grab(frames, isInit);
        }
        else
        {
            m_faceMonitorCallback[i]->grab({}, isInit);
        }
    }
}

GLuint FaceFinder::paint(const ScenePrimitives &scene, GLuint inputTexture)
{
    return inputTexture;
}

void FaceFinder::initColormap()
{
    // ##### Create a colormap ######
    cv::Mat1b colorsU8(1, 360);
    cv::Mat3b colorsU8C3;
    cv::Mat3f colors32FC3;
    for(int i = 0; i < 360; i++)
    {
        colorsU8(0,i) = uint8_t( 255.f * float(i)/(colorsU8.cols-1) + 0.5f );
    }
    
    cv::applyColorMap(colorsU8, colorsU8C3, cv::COLORMAP_HSV);
    colorsU8C3.convertTo(m_colors32FC3, CV_32FC3, 1.0/255.0);
}


void FaceFinder::computeAcf(const FrameInput &frame, bool doLuv, bool doDetection)
{
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);
    glDepthMask(GL_FALSE);
    
    m_acf->setDoLuvTransfer(doLuv);
    m_acf->setDoAcfTrasfer(doDetection);
    
    (*m_acf)(frame);
}

/*
 * Create full ACF pyramid on GPU
 */

std::shared_ptr<acf::Detector::Pyramid> FaceFinder::createAcfGpu(const FrameInput &frame, bool doDetection)
{
    computeAcf(frame, false, doDetection);
    
    std::shared_ptr<decltype(m_P)> P;
    cv::Mat acf = m_acf->getChannels(); // always trigger for gray output
    if(doDetection)
    {
        assert(acf.type() == CV_8UC1);
        assert(acf.channels() == 1);
        
        if(m_acf->getChannelStatus())
        {
            P = std::make_shared<decltype(m_P)>();
            fill(*P);
            
#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS
            cv::Mat channels = m_acf->getChannels();
            cv::imwrite("/tmp/acf_gpu.png", channels);
            logPyramid("/tmp/Pgpu.png", *P);
#endif
        }
    }
    
    return P;
}

/*
 * Create LUV images in OpenGL ES and uses this as input for
 * CPU based ACF pyramid construction.
 */

std::shared_ptr<acf::Detector::Pyramid> FaceFinder::createAcfCpu(const FrameInput &frame, bool doDetection)
{
    computeAcf(frame, true, doDetection);
    
    std::shared_ptr<decltype(m_P)> P;
    if(doDetection)
    {
        cv::Mat acf = m_acf->getChannels();
        assert(acf.type() == CV_8UC1);
        assert(acf.channels() == 1);
        
        P = std::make_shared<decltype(m_P)>();
        
        MatP LUVp = m_acf->getLuvPlanar();
        m_detector->setIsLuv(true);
        m_detector->setIsTranspose(true);
        m_detector->computePyramid(LUVp, *P);
        
#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS
        logPyramid("/tmp/Pcpu.png", *P);
#endif
    }

    return P;
}

/**
 * GPU preprocessing:
 * (1) FrameInpute -> texture -> ACF output image (P pyramid)
 * (2) Harris/Shi-Tomasi corners
 * (3) Resized grayscale image for face landmarks
 */

void FaceFinder::preprocess(const FrameInput &frame, ScenePrimitives &scene, bool doDetection)
{
    const auto tag = DRISHTI_LOCATION_SIMPLE;
    std::stringstream ss;
    core::ScopeTimeLogger scopeTimeLogger = [&](double t) { m_logger->info() << "TIMING:" << tag << ss.str() << "total=" << t; };

    if(m_doCpuACF)
    {
        scene.m_P = createAcfCpu(frame, doDetection);
    }
    else
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "acf=" << t << ";"; };
        scene.m_P = createAcfGpu(frame, doDetection);
    }
    
    // Flow pyramid currently unused:
    // auto flowPyramid = m_acf->getFlowPyramid();

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT
    if(m_acf->getFlowStatus())
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "flow=" << t << ";"; };
        cv::Size frameSize = uprightSize({frame.size.width, frame.size.height}, m_outputOrientation);
        cv::Mat4b ayxb = m_acf->getFlow();
        extractFlow(ayxb, frameSize, scene, 1.0f / m_acf->getFlowScale());
    }
#endif

    // ### Grayscale image ###
    if(m_doLandmarks)
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "gray=" << t << ";"; };
        scene.image() = m_acf->getGrayscale();
    }
}

void FaceFinder::fill(drishti::acf::Detector::Pyramid &P)
{
    m_acf->fill(P, m_P);
}

int FaceFinder::detect(const FrameInput &frame, ScenePrimitives &scene, bool doDetection)
{
    std::unique_lock<std::mutex> lock(m_mutex);
    
    core::ScopeTimeLogger scopeTimeLogger = [this](double t)
    {
        m_logger->info() << "FULL_CPU_PATH: " << t;
    };
    
    //m_logger->info() << "FaceFinder::detect() " << sBar;
    
    assert(scene.objects().size() == 0);

    if(m_detector && (!doDetection || scene.m_P))
    {
        // Test GPU ACF detection
        // Fill in ACF Pyramid structure
        std::vector<double> scores;

        if(doDetection)
        {
            core::ScopeTimeLogger scopeTimeLogger = [this](double t) { m_timerInfo.detectionTimeLogger(t); };
            
            (*m_detector)(*scene.m_P, scene.objects(), &scores);
            m_objects = std::make_pair(HighResolutionClock::now(), scene.objects());
        }
        else
        {
            scene.objects() = m_objects.second;
        }

        if(m_doLandmarks && scene.objects().size())
        {
#if DRISHTI_HCI_FACEFINDER_LOG_DETECTIONS
            {
                cv::Mat canvas({frame.size.width, frame.size.height}, CV_8UC4, frame.pixelBuffer);
                const auto objects = scene.objects() * m_ACFScale;
                for(auto &d : objects)
                {
                    cv::rectangle(canvas, d, {0,255,0}, 1, 8);
                }
                cv::imwrite("/tmp/detections.png", canvas);
            }
#endif

            auto &objects = scene.objects();
            std::vector<drishti::face::FaceModel> faces(objects.size());
            for(int i = 0; i < faces.size(); i++)
            {
                faces[i].roi = objects[i];
            }

            bool isDetection = true;

            cv::Mat1b gray = scene.image();

            const float Sdr = m_ACFScale /* acf->full */ * m_acf->getGrayscaleScale() /* full->gray */;
            cv::Matx33f Hdr(Sdr,0,0,0,Sdr,0,0,0,1); //  = cv::Matx33f::eye();
            drishti::face::FaceDetector::PaddedImage Ib(gray, {{0,0}, gray.size()});

            m_faceDetector->setDoIrisRefinement(true);
            m_faceDetector->setFaceStagesHint(8);
            m_faceDetector->setFace2StagesHint(4);
            m_faceDetector->setEyelidStagesHint(6);
            m_faceDetector->setIrisStagesHint(10);
            m_faceDetector->setIrisStagesRepetitionFactor(1);
            m_faceDetector->refine(Ib, faces, Hdr, isDetection);

            //float iod = cv::norm(faces[0].eyeFullR->irisEllipse.center - faces[0].eyeFullL->irisEllipse.center);

            // Scale faces from regression to level 0
            // The configuration sizes used in the ACF stacked channel image
            // are all upright, but the output texture used for the display
            // is still in the native (potentially rotated) coordinate system,
            // so we need to perform scaling wrt that.

            const float Srf = 1.0f / m_acf->getGrayscaleScale();
            cv::Matx33f H0(Srf,0,0,0,Srf,0,0,0,1);
            for(auto &f : faces)
            {
                f = H0 * f;
                
                // Tag each face w/ approximate distance:
                if(m_faceEstimator)
                {
                    if(f.eyeFullL.has && f.eyeFullR.has)
                    {
                        (*f.eyesCenter) = (*m_faceEstimator)(f);
                    }
                }
            }

            scene.faces() = faces;
        }
    }

    return 0;
}

void FaceFinder::updateEyes(GLuint inputTexId, const ScenePrimitives &scene)
{
    core::ScopeTimeLogger updateEyesLoge = [this](double t) { m_logger->info() << "FaceFinder::updateEyes=" << t; };
    
    if(scene.faces().size())
    {
        for(const auto &f : scene.faces())
        {
            m_eyeFilter->addFace(f);
        }
        
        // Trigger eye enhancer, triggers flash filter:
        m_eyeFilter->process(inputTexId, 1, GL_TEXTURE_2D);
        
        // Limit to points on iris:
        const cv::Size filteredEyeSize(m_flasher->getOutFrameW(), m_flasher->getOutFrameH());
        const auto &eyeWarps = m_eyeFilter->getEyeWarps();
        
        // Can use this to retrieve view of internal filters:
#define DRISHTI_VIEW_FLASH_OUTPUT 0
#if DRISHTI_VIEW_FLASH_OUTPUT
        if(m_flasher)
        {
            cv::Mat canvas = m_flasher->paint();
            if(!canvas.empty())
            {
                cv::imshow("flasher", canvas); cv::waitKey(0);
            }
        }
#endif

        if(m_doEyeFlow)
        { // Grab optical flow results:
            const auto flowSize = m_eyeFlowBgraInterface->getOutFrameSize();
            cv::Mat4b ayxb(flowSize.height, flowSize.width);
            m_eyeFlowBgraInterface->getResultData(ayxb.ptr());
            
            // Extract corners in bottom half of image
            cv::Mat1b corners;
            cv::extractChannel(ayxb, corners, 0);
            std::vector<FeaturePoint> features;
            extractPoints(corners, features, 1.f);

            m_eyeFlowField.clear();
            if(features.size())
            {
                cv::Matx33f Heye[2] =
                {
                    eyeWarps[0].H.inv() * transformation::normalize(ayxb.size()),
                    eyeWarps[1].H.inv() * transformation::normalize(ayxb.size())
                };
                
                std::vector<cv::Point2f> flow;
                for(const auto &f : features)
                {
                    // Extract flow:
                    const cv::Vec4b &pixel = ayxb(f.point.y, f.point.x);
                    cv::Point2f p(pixel[2], pixel[1]);
                    cv::Point2f d = (p * (2.0f / 255.0f)) - cv::Point2f(1.0f, 1.0f);
                    flow.push_back(d);
                    
                    cv::Point3f q3 = Heye[p.x > ayxb.cols/2] * f.point;
                    cv::Point2f q2(q3.x/q3.z, q3.y/q3.z);
                    m_eyeFlowField.emplace_back(q2.x, q2.y, d.x * 100.f, d.y * 100.f);
                }
                m_eyeMotion = -drishti::geometry::pointMedian(flow);
            }
        }
        
        { // Grab reflection points for eye tracking etc:
            core::ScopeTimeLogger scopeTimeLogger = [this](double t) { this->m_timerInfo.blobExtractionTimeLogger(t); };

            // Difference image on thread pool:
            EyeBlobJob difference(filteredEyeSize, eyeWarps);
            m_flasher->getHessianPeaksFromDifferenceImage()->getResultData(difference.filtered.ptr());
            auto differenceResult = m_threads->process([&]() { difference.run(); });
            
            { // Single image in current thread:
                EyeBlobJob single(filteredEyeSize, eyeWarps);
                m_flasher->getHessianPeaksFromSingleImage()->getResultData(single.filtered.ptr());
                single.run();
                m_eyePointsSingle = single.eyePoints;
            }
            
            computeGazePoints();

            differenceResult.get();            
            m_eyePointsDifference = difference.eyePoints;            
        }
    }
}

void FaceFinder::computeGazePoints()
{
    // Convert points to polar coordinates:
    m_gazePoints.clear();
    
    const auto &eyeWarps = m_eyeFilter->getEyeWarps();
    
    float total = 0.f;
    cv::Point2f mu;
    for(int i = 0; i < 2; i++)
    {
        for(const auto &p : m_eyePointsSingle[i])
        {
            // Find iris center relative to specular reflection:
            const auto &iris = eyeWarps[i].eye.irisEllipse;
            const cv::Point2f q = (iris.center - p.point);
            
            // Transform to unit circle:
            const float rho = cv::norm(q) / (iris.size.width * 0.5f);
            const cv::Point2f qi = cv::normalize(cv::Vec2f(q)) * rho;
            
            // Project points to unit circle:
            FeaturePoint gaze(qi, p.radius);
            m_gazePoints.push_back(gaze);
            
            // Compute mean point:
            const float weight = p.radius;
            mu += qi * weight;
            total += weight;
        }
    }
    if(total > 0.f)
    {
        mu *= (1.0 / total);
        m_logger->info() << "GAZE: " << mu;
    }
}

void FaceFinder::initTimeLoggers()
{
    m_timerInfo.detectionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.detectionTime = seconds;
    };
    m_timerInfo.regressionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.regressionTime = seconds;
    };
    m_timerInfo.eyeRegressionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.eyeRegressionTime = seconds;
    };
    m_timerInfo.acfProcessingTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.acfProcessingTime = seconds;
    };
    m_timerInfo.blobExtractionTimeLogger = [this](double seconds)
    {
        this->m_timerInfo.blobExtractionTime = seconds;
    };
    m_timerInfo.renderSceneTimeLogger  = [this](double seconds)
    {
        this->m_timerInfo.renderSceneTime = seconds;
    };
}

// #### init2 ####

void FaceFinder::init2(drishti::face::FaceDetectorFactory &resources)
{
    m_logger->info() << "FaceFinder::init2() " << sBar;
    m_logger->info() << resources;

    initTimeLoggers();

#if DRISHTI_HCI_FACEFINDER_DO_TRACKING
    // Insntiate a face detector w/ a tracking component:
    auto faceDetectorAndTracker = std::make_shared<drishti::face::FaceDetectorAndTracker>(resources);
    faceDetectorAndTracker->setMaxTrackAge(2.0);
    m_faceDetector = faceDetectorAndTracker;
#else
    m_faceDetector = std::make_shared<drishti::face::FaceDetector>(resources);
#endif
    m_faceDetector->setDoNMS(true);
    m_faceDetector->setInits(1);

    // Get weak ref to underlying ACF detector
    m_detector = dynamic_cast<drishti::acf::Detector*>(m_faceDetector->getDetector());

#if DRISHTI_HCI_FACEFINDER_DO_ACF_MODIFY
    if(m_detector)
    {
        // Perform modification
        drishti::acf::Detector::Modify dflt;
        dflt.cascThr = { "cascThr", -1.0 };
        dflt.cascCal = { "cascCal", +0.001 };
        m_detector->acfModify( dflt );
    }
#endif

    m_faceDetector->setDetectionTimeLogger(m_timerInfo.detectionTimeLogger);
    m_faceDetector->setRegressionTimeLogger(m_timerInfo.regressionTimeLogger);
    m_faceDetector->setEyeRegressionTimeLogger(m_timerInfo.eyeRegressionTimeLogger);

    {
        // FaceDetection mean:
        drishti::face::FaceModel faceDetectorMean = m_factory->getMeanFace();

        // We can change the regressor crop padding by doing a centered scaling of face features:
        if(0)
        {
            std::vector<cv::Point2f> centers
            {
                faceDetectorMean.getEyeLeftCenter(),
                faceDetectorMean.getEyeRightCenter(),
                *faceDetectorMean.noseTip
            };
            cv::Point2f center = core::centroid(centers);
            cv::Matx33f S(cv::Matx33f::diag({0.75, 0.75, 1.0}));
            cv::Matx33f T1(1,0,+center.x,0,1,+center.y,0,0,1);
            cv::Matx33f T2(1,0,-center.x,0,1,-center.y,0,0,1);
            cv::Matx33f H = T1 * S * T2;
            faceDetectorMean = H * faceDetectorMean;
        }

        m_faceDetector->setFaceDetectorMean(faceDetectorMean);
    }
}

// #### utilty: ####

std::ostream& operator<< (std::ostream& os, const FaceFinder::TimerInfo& info)
{
    double total = info.detectionTime + info.regressionTime + info.eyeRegressionTime;
    os  << " acf+=" << info.acfProcessingTime
        << " fd=" << info.detectionTime
        << " fr=" << info.regressionTime
        << " er=" << info.eyeRegressionTime
        << " blob=" << info.blobExtractionTime
        << " gl=" << info.renderSceneTime
        << " total=" << total;
    return os;
}

static int
getDetectionImageWidth(float objectWidthMeters, float fxPixels, float zMeters, float winSizePixels, float imageWidthPixels)
{
    // objectWidthPixels / fxPixels = objectWidthMeters / zMeters
    // objectWidthPixels = (fxPixels * objectWidthMeters) / zMeters
    const float objectWidthPixels = (fxPixels * objectWidthMeters) / zMeters;
    
    // objectWidthPixels / imageWidthPixels = winSizePixels / N
    // N = winSizePixels / (objectWidthPixels / imageWidthPixels);
    return winSizePixels / (objectWidthPixels / imageWidthPixels);
}

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT
static cv::Size uprightSize(const cv::Size &size, int orientation)
{
    cv::Size upSize = size;
    if((orientation / 90) % 2)
    {
        std::swap(upSize.width, upSize.height);
    }
    return upSize;
}


static void extractFlow(const cv::Mat4b &ayxb, const cv::Size &frameSize, ScenePrimitives &scene, float flowScale)
{
    // Compute size of flow image for just the lowest pyramid level:
    cv::Size flowSize = cv::Size2f(frameSize) * (1.0f / flowScale);
    cv::Rect flowRoi({0,0}, ayxb.size());
    flowRoi &= cv::Rect({0,0}, flowSize);

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER
    const int step = 2;
    scene.flow().reserve(scene.flow().size() + (ayxb.rows/step * ayxb.cols/step));

    for(int y = 0; y < ayxb.rows; y += step)
    {
        for(int x = 0; x < ayxb.cols; x += step)
        {
            // Extract flow:
            const cv::Vec4b &pixel = ayxb(y,x);
            cv::Point2f p(pixel[2], pixel[1]);
            cv::Point2f d = (p * (2.0f / 255.0f)) - cv::Point2f(1.0f, 1.0f);
            d *= 2.0; // additional scale for visualization
            scene.flow().emplace_back(cv::Vec4f(x, y, d.x, d.y) * flowScale);
        }
    }
#endif // DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER

    // {BGRA}, {RGBA}
    cv::Mat1b corners;
    cv::extractChannel(ayxb(flowRoi), corners, 0);
    
    std::vector<FeaturePoint> features;
    extractPoints(corners, features, flowScale);
    scene.corners() = {};
    scene.corners().reserve(features.size());
    for(const auto &f : features)
    {
        scene.corners().push_back(f.point);
    }
}

#endif // DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT

#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

static cv::Mat draw(const drishti::acf::Detector::Pyramid &pyramid)
{
    cv::Mat canvas;
    std::vector<cv::Mat> levels;
    for(int i = 0; i < pyramid.nScales; i++)
    {
        // Concatenate the transposed faces, so they are compatible with the GPU layout
        cv::Mat Ccpu;
        std::vector<cv::Mat> images;
        for(const auto &image : pyramid.data[i][0].get())
        {
            images.push_back(image.t());
        }
        cv::vconcat(images, Ccpu);
        
        // Instead of upright:
        //cv::vconcat(pyramid.data[i][0].get(), Ccpu);
        
        if(levels.size())
        {
            cv::copyMakeBorder(Ccpu, Ccpu, 0, levels.front().rows - Ccpu.rows, 0, 0, cv::BORDER_CONSTANT);
        }
        
        levels.push_back(Ccpu);
    }
    cv::hconcat(levels, canvas);
    return canvas;
}

static void logPyramid(const std::string &filename, const drishti::acf::Detector::Pyramid &P)
{
    cv::Mat canvas = draw(P);
    cv::imwrite(filename, canvas);
}

#endif // DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

DRISHTI_HCI_NAMESPACE_END
