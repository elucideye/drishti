/*!
 @file   drishti/hci/FaceFinder.cpp
 @author David Hirvonen
 @brief  Face detection and tracking class with GPU acceleration.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/hci/FaceFinder.h"
#include "drishti/hci/FaceFinderImpl.h"

#include "drishti/core/drishti_operators.h"      // cv::Size * float
#include "drishti/core/make_unique.h"            // make_unique<>
#include "drishti/core/timing.h"                 // ScopeTimeLogger
#include "drishti/face/FaceDetectorAndTracker.h" // *
#include "drishti/geometry/Primitives.h"         // operator
#include "drishti/geometry/motion.h"             // transformation::
#include "drishti/hci/EyeBlob.h"                 // EyeBlobJob

#include <functional>
#include <deque>

#include <spdlog/fmt/ostr.h>

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#  define TEXTURE_FORMAT_IS_RGBA 1
#else
#  define TEXTURE_FORMAT GL_BGRA
#  define TEXTURE_FORMAT_IS_RGBA 0
#endif
// clang-format on

static const char* sBar = "#################################################################";

// === utility ===

using drishti::face::operator*;
using drishti::core::operator*;

DRISHTI_HCI_NAMESPACE_BEGIN

static void chooseBest(std::vector<cv::Rect>& objects, std::vector<double>& scores);
static int getDetectionImageWidth(float, float, float, float, float);

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT
static cv::Size uprightSize(const cv::Size& size, int orientation);
static void extractFlow(const cv::Mat4b& ayxb, const cv::Size& frameSize, ScenePrimitives& scene, float flowScale = 1.f);
#endif

#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS
static cv::Mat draw(const drishti::acf::Detector::Pyramid& pyramid);
static void logPyramid(const std::string& filename, const drishti::acf::Detector::Pyramid& P);
#endif // DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

static ogles_gpgpu::Size2d convert(const cv::Size& size)
{
    return ogles_gpgpu::Size2d(size.width, size.height);
}

FaceFinder::FaceFinder(FaceDetectorFactoryPtr& factory, Settings& args, void* glContext)
{
    impl = drishti::core::make_unique<Impl>(factory, args, glContext);
}

void FaceFinder::setBrightness(float value)
{
    impl->brightness = value;
}

bool FaceFinder::doAnnotations() const
{
    return impl->doAnnotations;
}

void FaceFinder::setImageLogger(const ImageLogger& logger)
{
    impl->imageLogger = logger;
}

void FaceFinder::tryEnablePlatformOptimizations()
{
    ogles_gpgpu::ACF::tryEnablePlatformOptimizations();
}

std::unique_ptr<FaceFinder>
FaceFinder::create(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext)
{
    auto finder = drishti::core::make_unique<FaceFinder>(factory, settings, glContext);
    finder->initialize();
    return finder;
}

// Initialize outside of constructor for virtual calls:
void FaceFinder::initialize()
{
    impl->hasInit = true;
    init2(*impl->factory);
    init(impl->sensor->intrinsic().getSize());
}

FaceFinder::~FaceFinder()
{
    try
    {
        // If this has already been retrieved it will throw
        impl->scene.get(); // block on any abandoned calls
    }
    catch(...)
    {
        
    }
}

bool FaceFinder::needsDetection(const TimePoint& now) const
{
    double elapsed = std::chrono::duration_cast<std::chrono::duration<double>>(now - impl->objects.first).count();
    return (elapsed > impl->faceFinderInterval);
}

float FaceFinder::getMinDistance() const
{
    return impl->minDistanceMeters;
}

float FaceFinder::getMaxDistance() const
{
    return impl->maxDistanceMeters;
}

void FaceFinder::setDoCpuAcf(bool flag)
{
    impl->doCpuACF = flag;
}

bool FaceFinder::getDoCpuAcf() const
{
    return impl->doCpuACF;
}

void FaceFinder::setFaceFinderInterval(double interval)
{
    impl->faceFinderInterval = interval;
}

double FaceFinder::getFaceFinderInterval() const
{
    return impl->faceFinderInterval;
}

void FaceFinder::registerFaceMonitorCallback(FaceMonitor* callback)
{
    impl->faceMonitorCallback.push_back(callback);
}

void FaceFinder::dumpEyes(std::vector<cv::Mat4b>& frames, std::vector<std::array<eye::EyeModel, 2>>& eyes)
{
    impl->eyeFilter->dump(frames, eyes);
}

void FaceFinder::dumpFaces(std::vector<cv::Mat4b>& frames)
{
    if (impl->fifo->getBufferCount() == impl->fifo->getProcPasses().size())
    {
        frames.resize(impl->fifo->getBufferCount());
        for (int i = 0; i < frames.size(); i++)
        {
            auto* filter = (*impl->fifo)[i];

            cv::Size outSize(filter->getOutFrameW(), filter->getOutFrameH());
            frames[i].create(outSize.height, outSize.width);
            filter->getResultData(frames[i].ptr<uint8_t>());
        }
    }
}

int FaceFinder::computeDetectionWidth(const cv::Size& inputSizeUp) const
{
    // TODO: Add global constant and set limits on reasonable detection size
    const float faceWidthMeters = 0.120;
    const float fx = impl->sensor->intrinsic().m_fx;
    const float winSize = impl->detector->getWindowSize().width;
    return getDetectionImageWidth(faceWidthMeters, fx, impl->maxDistanceMeters, winSize, inputSizeUp.width);
}

// Side effect: set impl->pyramdSizes
void FaceFinder::initACF(const cv::Size& inputSizeUp)
{
    // ### ACF (Transpose) ###
    // Find the detection image width required for object detection at the max distance:
    const int detectionWidth = computeDetectionWidth(inputSizeUp);
    impl->ACFScale = float(inputSizeUp.width) / float(detectionWidth);

    // ACF implementation uses reduce resolution transposed image:
    cv::Size detectionSize = inputSizeUp * (1.0f / impl->ACFScale);
    cv::Mat I(detectionSize.width, detectionSize.height, CV_32FC3, cv::Scalar::all(0));

    MatP Ip(I);
    impl->detector->computePyramid(Ip, impl->P);

    impl->pyramidSizes.resize(impl->P.nScales);
    std::vector<ogles_gpgpu::Size2d> sizes(impl->P.nScales);
    for (int i = 0; i < impl->P.nScales; i++)
    {
        const auto size = impl->P.data[i][0][0].size();
        sizes[i] = { size.width * 4, size.height * 4 }; // undo ACF binning x4
        impl->pyramidSizes[i] = { size.width * 4, size.height * 4 };

        // CPU processing works with tranposed images for col-major storage assumption.
        // Undo that here to map to row major representation.  Perform this step
        // to make transpose operation explicit.
        std::swap(sizes[i].width, sizes[i].height);
        std::swap(impl->pyramidSizes[i].width, impl->pyramidSizes[i].height);
    }

    const int grayWidth = impl->doLandmarks ? std::min(inputSizeUp.width, impl->landmarksWidth) : 0;
    const int flowWidth = impl->doFlow ? impl->flowWidth : 0;
    const auto& pChns = impl->detector->opts.pPyramid->pChns;
    const auto featureKind = ogles_gpgpu::getFeatureKind(*pChns);

    CV_Assert(featureKind != ogles_gpgpu::ACF::kUnknown);

    const ogles_gpgpu::Size2d size(inputSizeUp.width, inputSizeUp.height);
    impl->acf = std::make_shared<ogles_gpgpu::ACF>(impl->glContext, size, sizes, featureKind, grayWidth, flowWidth, impl->debugACF);
    impl->acf->setRotation(impl->outputOrientation);
    impl->acf->setLogger(impl->logger);
}

// ### Fifo ###
void FaceFinder::initFIFO(const cv::Size& inputSize, std::size_t n)
{
    impl->fifo = std::make_shared<ogles_gpgpu::FifoProc>(n);
    impl->fifo->init(inputSize.width, inputSize.height, INT_MAX, false);
    impl->fifo->createFBOTex(false);
}

void FaceFinder::initBlobFilter()
{
    // ### Blobs ###
    assert(impl->eyeFilter.get());
    impl->blobFilter = std::make_shared<ogles_gpgpu::BlobFilter>();
    impl->blobFilter->init(128, 64, INT_MAX, false);
    impl->blobFilter->createFBOTex(false);

    // ### Send eye images into the flash filter ###
    impl->eyeFilter->getOutputFilter()->add(impl->blobFilter->getInputFilter());
}

void FaceFinder::initIris(const cv::Size& size)
{
    // ### Ellipsopolar warper ####
    for (int i = 0; i < 2; i++)
    {
        impl->ellipsoPolar[i] = std::make_shared<ogles_gpgpu::EllipsoPolarWarp>();
        impl->ellipsoPolar[i]->setOutputSize(size.width, size.height);
    }
}

void FaceFinder::initEyeEnhancer(const cv::Size& inputSizeUp, const cv::Size& eyesSize)
{
    // ### Eye enhancer ###
    const auto mode = ogles_gpgpu::EyeFilter::kMean3;
    const float cutoff = 0.5;

    impl->eyeFilter = std::make_shared<ogles_gpgpu::EyeFilter>(convert(eyesSize), mode, cutoff);
    impl->eyeFilter->setAutoScaling(true);
    impl->eyeFilter->setOutputSize(eyesSize.width, eyesSize.height);

    if (impl->doIris)
    {
        // Add a callback to retrieve updated eye models automatically:
        cv::Matx33f N = transformation::scale(0.5, 0.5) * transformation::translate(1.f, 1.f);
        for (int i = 0; i < 2; i++)
        {
            std::function<drishti::eye::EyeWarp()> eyeDelegate = [&, N, i]() {
                auto eye = impl->eyeFilter->getEyeWarps()[i];
                eye.eye = N * eye.H * eye.eye;
                return eye;
            };
            impl->ellipsoPolar[i]->addEyeDelegate(eyeDelegate);
            impl->eyeFilter->add(impl->ellipsoPolar[i].get());
        }
    }

    if (impl->doEyeFlow)
    { // optical flow for eyes:
        impl->eyeFlow = std::make_shared<ogles_gpgpu::FlowOptPipeline>(0.004, 1.0, false);
#if TEXTURE_FORMAT_IS_RGBA
        impl->eyeFlowBgra = std::make_shared<ogles_gpgpu::SwizzleProc>();
        impl->eyeFlow->add(impl->eyeFlowBgra.get());
        impl->eyeFlowBgraInterface = impl->eyeFlowBgra.get();
#else
        impl->eyeFlowBgraInterface = impl->eyeFlow.get();
#endif

        impl->eyeFilter->add(impl->eyeFlow.get());
    }

    impl->eyeFilter->prepare(inputSizeUp.width, inputSizeUp.height, (GLenum)GL_RGBA);
}

void FaceFinder::initPainter(const cv::Size& /* inputSizeUp */)
{
    impl->logger->info("Init painter");
}

void FaceFinder::init(const cv::Size& inputSize)
{
    //impl->logger->set_level(spdlog::level::err);

    impl->start = HighResolutionClock::now();

    auto inputSizeUp = inputSize;
    bool hasTranspose = ((impl->outputOrientation / 90) % 2);
    if (hasTranspose)
    {
        std::swap(inputSizeUp.width, inputSizeUp.height);
    }

    impl->faceEstimator = std::make_shared<drishti::face::FaceModelEstimator>(*impl->sensor);

    initColormap();
    initACF(inputSizeUp);     // initialize ACF first (configure opengl platform extensions)
    initFIFO(inputSizeUp, 3); // keep last 3 frames
    initPainter(inputSizeUp); // {inputSizeUp.width/4, inputSizeUp.height/4}

    if (impl->doIris)
    {
        initIris({ 640, 240 });
    }

    // Must initial eye filter before blobFilter:
    initEyeEnhancer(inputSizeUp, impl->eyesSize);

    if (impl->doBlobs)
    {
        // Must initialize blobFilter after eye filter:
        initBlobFilter();
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

GLuint FaceFinder::operator()(const FrameInput& frame1)
{
    assert(impl->hasInit);
    assert(impl->sensor->intrinsic().getSize() == cv::Size(frame1.size.width, frame1.size.height));

    std::string methodName = DRISHTI_LOCATION_SIMPLE;
    core::ScopeTimeLogger faceFinderTimeLogger = [this, methodName](double elapsed) {
        if (impl->logger)
        {
            impl->logger->info("TIMING:{} : {} full={}", methodName, impl->timerInfo, elapsed);
        }
    };

    // Get current timestamp
    const auto& now = faceFinderTimeLogger.getTime();

    const bool doDetection = needsDetection(now);

    impl->frameIndex++; // increment frame index

    // Run GPU based processing on current thread and package results as a task for CPU
    // processing so that it will be available on the next frame.  This method will compute
    // ACF output using shaders on the GPU, and may optionally extract other GPU related
    // features.
    ScenePrimitives scene1(impl->frameIndex), scene0, *outputScene = nullptr; // time: n+1 and n

    { // *timing*
        core::ScopeTimeLogger preprocessTimeLogger = [this](double t) { impl->timerInfo.acfProcessingTime = t; };
        preprocess(frame1, scene1, doDetection);
        
        //detectOnly(scene1, doDetection); // optionally try detection on current thread
    }

    // Initialize input texture with ACF upright texture:
    GLuint texture1 = impl->acf->first()->getOutputTexId(), texture0 = 0, outputTexture = 0;

    if (impl->threads)
    {
        // Retrieve the previous frame and scene
        if ((impl->fifo->getBufferCount() > 0) && doAnnotations())
        {
            {
                // Retrieve the previous frame (latency == 1)
                // from our N frame FIFO.
                core::ScopeTimeLogger paintTimeLogger = [this](double t) { impl->logger->info("WAITING: {}", t); };
                scene0 = impl->scene.get();                     // scene n-1
                texture0 = (*impl->fifo)[-1]->getOutputTexId(); // texture n-1
            }

            updateEyes(texture0, scene0); // update the eye texture

            { // Explicit output variable configuration:
                core::ScopeTimeLogger glTimeLogger = [this](double t) { impl->timerInfo.renderSceneTimeLogger(t); };
                outputTexture = paint(scene0, texture0);
                outputScene = &scene0;
            }
        }
        else
        {
            outputTexture = texture1;
            outputScene = &scene0; // empty
        }

        // Enque the current frame and scene for CPU processing so
        // that results will be available for the next step (see above).
        impl->scene = impl->threads->process([scene1, frame1, doDetection, this]() {
            ScenePrimitives sceneOut = scene1;
            detect(frame1, sceneOut, doDetection);
            if (doAnnotations())
            {
                // prepare line drawings for rendering while gpu is busy
                sceneOut.draw(impl->renderFaces, impl->renderPupils, impl->renderCorners);

                // TODO: Eye contours for current scene can be created
                // for next frame here on the CPU thread.
            }
            return sceneOut;
        });
    }
    else
    {
        detect(frame1, scene1, doDetection);

        if (doAnnotations())
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
    impl->fifo->useTexture(texture1, 1);
    impl->fifo->render();

    // Clear face motion estimate, update window:
    impl->faceMotion = { 0.f, 0.f, 0.f };
    impl->scenePrimitives.push_front(*outputScene);

    if (impl->scenePrimitives.size() >= 3)
    {
        impl->scenePrimitives.pop_back();
    }

    if (impl->scenePrimitives.size() >= 2)
    {
        if (impl->scenePrimitives[0].faces().size() && impl->scenePrimitives[1].faces().size())
        {
            const auto& face0 = impl->scenePrimitives[0].faces()[0];
            const auto& face1 = impl->scenePrimitives[1].faces()[0];
            const cv::Point3f delta = (*face0.eyesCenter - *face1.eyesCenter);
            impl->faceMotion = delta; // active face motion
        }
    }

    try
    {
        this->notifyListeners(*outputScene, now, impl->fifo->isFull());
    }
    catch (...)
    {
    }

    return outputTexture;
}

static bool hasValidFaceRequest(FaceMonitor& monitor, const ScenePrimitives& scene, const FaceMonitor::TimePoint& now)
{
    for (auto& face : scene.faces())
    {
        if (monitor.isValid((*face.eyesCenter), now))
        {
            return true;
        }
    }
    return false;
}

// Query list of listeners for valid face image
bool FaceFinder::hasValidFaceRequest(const ScenePrimitives& scene, const TimePoint& now) const
{
    for (auto& callback : impl->faceMonitorCallback)
    {
        if (::drishti::hci::hasValidFaceRequest(*callback, scene, now))
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

void FaceFinder::notifyListeners(const ScenePrimitives& scene, const TimePoint& now, bool isInit)
{
    // Perform optional frame grabbing
    // NOTE: This must occur in the main OpenGL thread:
    std::vector<FaceMonitor::FaceImage> frames;

    // Build a list of active requests:
    bool hasActive = false;
    std::vector<bool> isActive(impl->faceMonitorCallback.size(), false);

    if (scene.faces().size())
    {
        // 1) If any active face request is satisifed grab a frame+face buffer:
        for (int i = 0; i < impl->faceMonitorCallback.size(); i++)
        {
            auto& callback = impl->faceMonitorCallback[i];
            isActive[i] = ::drishti::hci::hasValidFaceRequest(*callback, scene, now);
            hasActive |= isActive[i];
        }

        if (hasActive)
        {
            // ### collect face images ###
            std::vector<cv::Mat4b> faces;
            dumpFaces(faces);

            if (faces.size())
            {
                frames.resize(faces.size());
                for (int i = 0; i < frames.size(); i++)
                {
                    frames[i].image = faces[i];
                }
                // Tag active face image with model
                frames[0].faceModels = scene.faces();

                // ### collect eye images ###
                std::vector<cv::Mat4b> eyes;
                std::vector<std::array<eye::EyeModel, 2>> eyePairs;
                dumpEyes(eyes, eyePairs);
                for (int i = 0; i < std::min(eyes.size(), faces.size()); i++)
                {
                    frames[i].eyes = eyes[i];
                }

                // ### Add the eye difference image ###
                cv::Mat4b filtered(impl->eyeFilter->getOutFrameH(), impl->eyeFilter->getOutFrameW());
                impl->eyeFilter->getResultData(filtered.ptr());
                frames[0].extra = filtered;
            }
        }
    }

    // 3) Provide face images as requested:
    for (int i = 0; i < impl->faceMonitorCallback.size(); i++)
    {
        if (isActive[i])
        {
            impl->faceMonitorCallback[i]->grab(frames, isInit);
        }
        else
        {
            impl->faceMonitorCallback[i]->grab({}, isInit);
        }
    }
}

GLuint FaceFinder::paint(const ScenePrimitives& scene, GLuint inputTexture)
{
    return inputTexture;
}

void FaceFinder::initColormap()
{
    // ##### Create a colormap ######
    cv::Mat1b colorsU8(1, 360);
    cv::Mat3b colorsU8C3;
    cv::Mat3f colors32FC3;
    for (int i = 0; i < 360; i++)
    {
        colorsU8(0, i) = uint8_t(255.f * float(i) / (colorsU8.cols - 1) + 0.5f);
    }

    cv::applyColorMap(colorsU8, colorsU8C3, cv::COLORMAP_HSV);
    colorsU8C3.convertTo(impl->colors32FC3, CV_32FC3, 1.0 / 255.0);
}

void FaceFinder::computeAcf(const FrameInput& frame, bool doLuv, bool doDetection)
{
    glDisable(GL_BLEND);
    glDisable(GL_DEPTH_TEST);
    glDisable(GL_DITHER);
    glDepthMask(GL_FALSE);

    impl->acf->setDoLuvTransfer(doLuv);
    impl->acf->setDoAcfTrasfer(doDetection);

    (*impl->acf)(frame);
}

/*
 * Create full ACF pyramid on GPU
 */

std::shared_ptr<acf::Detector::Pyramid> FaceFinder::createAcfGpu(const FrameInput& frame, bool doDetection)
{
    computeAcf(frame, false, doDetection);

    std::shared_ptr<decltype(impl->P)> P;
    cv::Mat acf = impl->acf->getChannels(); // always trigger for gray output
    if (doDetection)
    {
        assert(acf.type() == CV_8UC1);
        assert(acf.channels() == 1);

        if (impl->acf->getChannelStatus())
        {
            P = std::make_shared<decltype(impl->P)>();
            fill(*P);

#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS
            cv::Mat channels = impl->acf->getChannels();
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

std::shared_ptr<acf::Detector::Pyramid> FaceFinder::createAcfCpu(const FrameInput& frame, bool doDetection)
{
    computeAcf(frame, true, doDetection);

    std::shared_ptr<decltype(impl->P)> P;
    if (doDetection)
    {
        cv::Mat acf = impl->acf->getChannels();
        assert(acf.type() == CV_8UC1);
        assert(acf.channels() == 1);

        P = std::make_shared<decltype(impl->P)>();

        MatP LUVp = impl->acf->getLuvPlanar();
        impl->detector->setIsLuv(true);
        impl->detector->setIsTranspose(true);
        impl->detector->computePyramid(LUVp, *P);

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

void FaceFinder::preprocess(const FrameInput& frame, ScenePrimitives& scene, bool doDetection)
{
    const auto tag = DRISHTI_LOCATION_SIMPLE;
    std::stringstream ss;
    core::ScopeTimeLogger scopeTimeLogger = [&](double t) { impl->logger->info("TIMING:{}{}total={}", tag, ss.str(), t); };

    if (impl->doCpuACF)
    {
        scene.m_P = createAcfCpu(frame, doDetection);
    }
    else
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "acf=" << t << ";"; };
        scene.m_P = createAcfGpu(frame, doDetection);
    }

    // Flow pyramid currently unused:
    // auto flowPyramid = impl->acf->getFlowPyramid();
#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT
    if (impl->acf->getFlowStatus())
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "flow=" << t << ";"; };
        cv::Size frameSize = uprightSize({ frame.size.width, frame.size.height }, impl->outputOrientation);
        cv::Mat4b ayxb = impl->acf->getFlow();
        extractFlow(ayxb, frameSize, scene, 1.0f / impl->acf->getFlowScale());
    }
#endif

    // ### Grayscale image ###
    if (impl->doLandmarks)
    {
        core::ScopeTimeLogger scopeTimeLogger = [&](double t) { ss << "gray=" << t << ";"; };
        scene.image() = impl->acf->getGrayscale();

        if (impl->imageLogger)
        {
            (impl->imageLogger)(scene.image());
        }
    }
}

void FaceFinder::fill(drishti::acf::Detector::Pyramid& P)
{
    impl->acf->fill(P, impl->P);
}

int FaceFinder::detectOnly(ScenePrimitives& scene, bool doDetection)
{
    // Fill in ACF Pyramid structure.
    // Check to see if detection was already computed
    if (doDetection)
    {
        core::ScopeTimeLogger scopeTimeLogger = [this](double t) { impl->timerInfo.detectionTimeLogger(t); };
        std::vector<double> scores;
        (*impl->detector)(*scene.m_P, scene.objects(), &scores);
        if (impl->doNMSGlobal)
        {
            chooseBest(scene.objects(), scores);
        }
        impl->objects = std::make_pair(HighResolutionClock::now(), scene.objects());
    }
    else
    {
        scene.objects() = impl->objects.second;
    }
    
    return scene.objects().size();
}

int FaceFinder::detect(const FrameInput& frame, ScenePrimitives& scene, bool doDetection)
{
    //impl->logger->set_level(spdlog::level::off);
    
    core::ScopeTimeLogger scopeTimeLogger = [this](double t) {
        impl->logger->info("FULL_CPU_PATH: {}", t);
    };

    //assert(scene.objects().size() == 0); /* can be precomputed now */

    if (impl->detector && (!doDetection || scene.m_P))
    {
        if(!scene.objects().size())
        {
            detectOnly(scene, doDetection);
        }
        if (impl->doLandmarks && scene.objects().size())
        {
#if DRISHTI_HCI_FACEFINDER_LOG_DETECTIONS
            {
                cv::Mat canvas({ frame.size.width, frame.size.height }, CV_8UC4, frame.pixelBuffer);
                const auto objects = scene.objects() * impl->ACFScale;
                for (auto& d : objects)
                {
                    cv::rectangle(canvas, d, { 0, 255, 0 }, 1, 8);
                }
                cv::imwrite("/tmp/detections.png", canvas);
            }
#endif

            auto& objects = scene.objects();
            std::vector<drishti::face::FaceModel> faces(objects.size());
            for (int i = 0; i < faces.size(); i++)
            {
                faces[i].roi = objects[i];
            }

            bool isDetection = true;

            cv::Mat1b gray = scene.image();

            //impl->imageLogger(gray);

            const float Sdr = impl->ACFScale /* acf->full */ * impl->acf->getGrayscaleScale() /* full->gray */;
            cv::Matx33f Hdr(Sdr, 0, 0, 0, Sdr, 0, 0, 0, 1); //  = cv::Matx33f::eye();
            drishti::face::FaceDetector::PaddedImage Ib(gray, { { 0, 0 }, gray.size() });

            impl->faceDetector->setDoIrisRefinement(true);
            impl->faceDetector->setFaceStagesHint(8);
            impl->faceDetector->setFace2StagesHint(4);
            impl->faceDetector->setEyelidStagesHint(6);
            impl->faceDetector->setIrisStagesHint(10);
            impl->faceDetector->setIrisStagesRepetitionFactor(1);
            impl->faceDetector->refine(Ib, faces, Hdr, isDetection);

            impl->logger->info("Face image {} x {}", Ib.Ib.rows, Ib.Ib.cols);

            //float iod = cv::norm(faces[0].eyeFullR->irisEllipse.center - faces[0].eyeFullL->irisEllipse.center);

            // Scale faces from regression to level 0
            // The configuration sizes used in the ACF stacked channel image
            // are all upright, but the output texture used for the display
            // is still in the native (potentially rotated) coordinate system,
            // so we need to perform scaling wrt that.

            const float Srf = 1.0f / impl->acf->getGrayscaleScale();
            cv::Matx33f H0(Srf, 0, 0, 0, Srf, 0, 0, 0, 1);
            for (auto& f : faces)
            {
                f = H0 * f;

                // Tag each face w/ approximate distance:
                if (impl->faceEstimator)
                {
                    if (f.eyeFullL.has && f.eyeFullR.has)
                    {
                        (*f.eyesCenter) = (*impl->faceEstimator)(f);
                    }
                }
            }

            scene.faces() = faces;
        }
    }

    return 0;
}

void FaceFinder::updateEyes(GLuint inputTexId, const ScenePrimitives& scene)
{
    core::ScopeTimeLogger updateEyesLoge = [this](double t) { impl->logger->info("FaceFinder::updateEyes={}", t); };

    if (scene.faces().size())
    {
        for (const auto& f : scene.faces())
        {
            impl->eyeFilter->addFace(f);
        }

        // Trigger eye enhancer, triggers flash filter:
        impl->eyeFilter->process(inputTexId, 1, GL_TEXTURE_2D);

        // Limit to points on iris:
        const auto& eyeWarps = impl->eyeFilter->getEyeWarps();
// Can use this to retrieve view of internal filters:
#define DRISHTI_VIEW_BLOBS_OUTPUT 0
#if DRISHTI_VIEW_BLOBS_OUTPUT
        if (impl->blobFilter)
        {
            cv::Mat canvas = impl->blobFilter->paint();
            if (!canvas.empty())
            {
                cv::imshow("blobFilter", canvas);
                cv::waitKey(0);
            }
        }
#endif

        if (impl->doEyeFlow)
        { // Grab optical flow results:
            const auto flowSize = impl->eyeFlowBgraInterface->getOutFrameSize();
            cv::Mat4b ayxb(flowSize.height, flowSize.width);
            impl->eyeFlowBgraInterface->getResultData(ayxb.ptr());

            // Extract corners in bottom half of image
            cv::Mat1b corners;
            cv::extractChannel(ayxb, corners, 0);
            std::vector<FeaturePoint> features;
            extractPoints(corners, features, 1.f);

            impl->eyeFlowField.clear();
            if (features.size())
            {
                cv::Matx33f Heye[2] = {
                    eyeWarps[0].H.inv() * transformation::normalize(ayxb.size()),
                    eyeWarps[1].H.inv() * transformation::normalize(ayxb.size())
                };

                std::vector<cv::Point2f> flow;
                for (const auto& f : features)
                {
                    // Extract flow:
                    const cv::Vec4b& pixel = ayxb(f.point.y, f.point.x);
                    cv::Point2f p(pixel[2], pixel[1]);
                    cv::Point2f d = (p * (2.0f / 255.0f)) - cv::Point2f(1.0f, 1.0f);
                    flow.push_back(d);

                    cv::Point3f q3 = Heye[p.x > ayxb.cols / 2] * f.point;
                    cv::Point2f q2(q3.x / q3.z, q3.y / q3.z);
                    impl->eyeFlowField.emplace_back(q2.x, q2.y, d.x * 100.f, d.y * 100.f);
                }
                impl->eyeMotion = -drishti::geometry::pointMedian(flow);
            }
        }

        if (impl->blobFilter)
        { // Grab reflection points for eye tracking etc:
            core::ScopeTimeLogger scopeTimeLogger = [this](double t) { this->impl->timerInfo.blobExtractionTimeLogger(t); };

            const cv::Size filteredEyeSize(impl->blobFilter->getOutFrameW(), impl->blobFilter->getOutFrameH());
            EyeBlobJob single(filteredEyeSize, eyeWarps);
            impl->blobFilter->getHessianPeaks()->getResultData(single.filtered.ptr());
            single.run();
            impl->eyePoints = single.eyePoints;

            computeGazePoints();
        }
    }
}

void FaceFinder::computeGazePoints()
{
    // Convert points to polar coordinates:
    impl->gazePoints.clear();

    const auto& eyeWarps = impl->eyeFilter->getEyeWarps();

    float total = 0.f;
    cv::Point2f mu;
    for (int i = 0; i < 2; i++)
    {
        for (const auto& p : impl->eyePoints[i])
        {
            // Find iris center relative to specular reflection:
            const auto& iris = eyeWarps[i].eye.irisEllipse;
            const cv::Point2f q = (iris.center - p.point);

            // Transform to unit circle:
            const float rho = cv::norm(q) / (iris.size.width * 0.5f);
            const cv::Point2f qi = cv::normalize(cv::Vec2f(q)) * rho;

            // Project points to unit circle:
            FeaturePoint gaze(qi, p.radius);
            impl->gazePoints.push_back(gaze);

            // Compute mean point:
            const float weight = p.radius;
            mu += qi * weight;
            total += weight;
        }
    }
    if (total > 0.f)
    {
        mu *= (1.0 / total);
        impl->logger->info("GAZE: {:f}", mu);
    }
}

void FaceFinder::initTimeLoggers()
{
    impl->timerInfo.detectionTimeLogger = [this](double seconds) {
        this->impl->timerInfo.detectionTime = seconds;
    };
    impl->timerInfo.regressionTimeLogger = [this](double seconds) {
        this->impl->timerInfo.regressionTime = seconds;
    };
    impl->timerInfo.eyeRegressionTimeLogger = [this](double seconds) {
        this->impl->timerInfo.eyeRegressionTime = seconds;
    };
    impl->timerInfo.acfProcessingTimeLogger = [this](double seconds) {
        this->impl->timerInfo.acfProcessingTime = seconds;
    };
    impl->timerInfo.blobExtractionTimeLogger = [this](double seconds) {
        this->impl->timerInfo.blobExtractionTime = seconds;
    };
    impl->timerInfo.renderSceneTimeLogger = [this](double seconds) {
        this->impl->timerInfo.renderSceneTime = seconds;
    };
}

// #### init2 ####

void FaceFinder::init2(drishti::face::FaceDetectorFactory& resources)
{
    impl->logger->info("FaceFinder::init2() {}", sBar);
    impl->logger->info("{}", resources);

    initTimeLoggers();

#if DRISHTI_HCI_FACEFINDER_DO_TRACKING
    // Insntiate a face detector w/ a tracking component:
    auto faceDetectorAndTracker = std::make_shared<drishti::face::FaceDetectorAndTracker>(resources);
    faceDetectorAndTracker->setMaxTrackAge(2.0);
    impl->faceDetector = faceDetectorAndTracker;
#else
    impl->faceDetector = std::make_shared<drishti::face::FaceDetector>(resources);
#endif
    impl->faceDetector->setDoNMSGlobal(impl->doNMSGlobal); // single detection only
    impl->faceDetector->setDoNMS(true);
    impl->faceDetector->setInits(1);

    // Get weak ref to underlying ACF detector
    impl->detector = dynamic_cast<drishti::acf::Detector*>(impl->faceDetector->getDetector());

#if DRISHTI_HCI_FACEFINDER_DO_ACF_MODIFY
    if (impl->detector)
    {
        // Perform modification
        drishti::acf::Detector::Modify dflt;
        dflt.cascThr = { "cascThr", -1.0 };
        dflt.cascCal = { "cascCal", -0.004 };
        impl->detector->acfModify(dflt);
    }
#endif

    impl->faceDetector->setDetectionTimeLogger(impl->timerInfo.detectionTimeLogger);
    impl->faceDetector->setRegressionTimeLogger(impl->timerInfo.regressionTimeLogger);
    impl->faceDetector->setEyeRegressionTimeLogger(impl->timerInfo.eyeRegressionTimeLogger);

    {
        // FaceDetection mean:
        drishti::face::FaceModel faceDetectorMean = impl->factory->getMeanFace();

        // We can change the regressor crop padding by doing a centered scaling of face features:
        if (impl->regressorCropScale > 0.f)
        {
            std::vector<cv::Point2f> centers{
                faceDetectorMean.getEyeLeftCenter(),
                faceDetectorMean.getEyeRightCenter(),
                *faceDetectorMean.noseTip
            };
            cv::Point2f center = core::centroid(centers);
            cv::Matx33f S(cv::Matx33f::diag({ impl->regressorCropScale, impl->regressorCropScale, 1.0 }));
            cv::Matx33f T1(1, 0, +center.x, 0, 1, +center.y, 0, 0, 1);
            cv::Matx33f T2(1, 0, -center.x, 0, 1, -center.y, 0, 0, 1);
            cv::Matx33f H = T1 * S * T2;
            faceDetectorMean = H * faceDetectorMean;
        }

        impl->faceDetector->setFaceDetectorMean(faceDetectorMean);
    }
}

// #### utilty: ####

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

std::ostream& operator<<(std::ostream& os, const FaceFinder::TimerInfo& info)
{
    double total = info.detectionTime + info.regressionTime + info.eyeRegressionTime;
    os << " acf+=" << info.acfProcessingTime
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
static cv::Size uprightSize(const cv::Size& size, int orientation)
{
    cv::Size upSize = size;
    if ((orientation / 90) % 2)
    {
        std::swap(upSize.width, upSize.height);
    }
    return upSize;
}

static void extractFlow(const cv::Mat4b& ayxb, const cv::Size& frameSize, ScenePrimitives& scene, float flowScale)
{
    // Compute size of flow image for just the lowest pyramid level:
    cv::Size flowSize = cv::Size2f(frameSize) * (1.0f / flowScale);
    cv::Rect flowRoi({ 0, 0 }, ayxb.size());
    flowRoi &= cv::Rect({ 0, 0 }, flowSize);

#if DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER
    const int step = 2;
    scene.flow().reserve(scene.flow().size() + (ayxb.rows / step * ayxb.cols / step));

    for (int y = 0; y < ayxb.rows; y += step)
    {
        for (int x = 0; x < ayxb.cols; x += step)
        {
            // Extract flow:
            const cv::Vec4b& pixel = ayxb(y, x);
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
    for (const auto& f : features)
    {
        scene.corners().push_back(f.point);
    }
}

#endif // DRISHTI_HCI_FACEFINDER_DO_FLOW_QUIVER || DRISHTI_HCI_FACEFINDER_DO_CORNER_PLOT

#if DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

static cv::Mat draw(const drishti::acf::Detector::Pyramid& pyramid)
{
    cv::Mat canvas;
    std::vector<cv::Mat> levels;
    for (int i = 0; i < pyramid.nScales; i++)
    {
        // Concatenate the transposed faces, so they are compatible with the GPU layout
        cv::Mat Ccpu;
        std::vector<cv::Mat> images;
        for (const auto& image : pyramid.data[i][0].get())
        {
            images.push_back(image.t());
        }
        cv::vconcat(images, Ccpu);

        // Instead of upright:
        //cv::vconcat(pyramid.data[i][0].get(), Ccpu);

        if (levels.size())
        {
            cv::copyMakeBorder(Ccpu, Ccpu, 0, levels.front().rows - Ccpu.rows, 0, 0, cv::BORDER_CONSTANT);
        }

        levels.push_back(Ccpu);
    }
    cv::hconcat(levels, canvas);
    return canvas;
}

static void logPyramid(const std::string& filename, const drishti::acf::Detector::Pyramid& P)
{
    cv::Mat canvas = draw(P);
    cv::imwrite(filename, canvas);
}

#endif // DRISHTI_HCI_FACEFINDER_DEBUG_PYRAMIDS

DRISHTI_HCI_NAMESPACE_END
