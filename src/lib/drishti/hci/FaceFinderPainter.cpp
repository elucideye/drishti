/*! -*-c++-*-
  @file   drishti/hci/FaceFinderPainter.cpp
  @author David Hirvonen
  @brief  Face tracking class with OpenGL texture drawing

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/FaceFinderPainter.h"
#include "drishti/hci/FaceFinderImpl.h"
#include "drishti/hci/gpu/FacePainter.h"
#include "drishti/hci/gpu/LineDrawing.hpp"
#include "drishti/hci/gpu/BlobFilter.h"
#include "drishti/hci/gpu/GLCircle.h"
#include "drishti/eye/gpu/EllipsoPolarWarp.h"
#include "drishti/core/drishti_operators.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/timing.h"
#include "drishti/core/Logger.h"
#include "drishti/core/Parallel.h"
#include "drishti/geometry/motion.h"

#include <acf/GPUACF.h>

#include "ogles_gpgpu/common/types.h"
#include "ogles_gpgpu/common/proc/transform.h"
#include "ogles_gpgpu/common/gl/memtransfer_optimized.h"

#include <memory>

using drishti::face::operator*;
using drishti::core::operator*;

struct MethodLog
{
    MethodLog(const std::string& name)
        : name(name)
    {
    }
    std::string name;
    std::stringstream ss;
};

DRISHTI_HCI_NAMESPACE_BEGIN

static void getImage(ogles_gpgpu::ProcInterface& proc, FaceFinderPainter::FrameDelegate& callback)
{
    if (dynamic_cast<ogles_gpgpu::MemTransferOptimized*>(proc.getMemTransferObj()))
    {
        // clag-format off
        ogles_gpgpu::MemTransfer::FrameDelegate delegate = [&](const ogles_gpgpu::Size2d& size, const void* pixels, size_t bytesPerRow) {
            callback(cv::Mat(size.height, size.width, CV_8UC4, (void*)pixels, bytesPerRow));
        };
        // clag-format on
        proc.getResultData(delegate);
    }
    else
    {
        cv::Mat frame(proc.getOutFrameH(), proc.getOutFrameW(), CV_8UC4);
        proc.getResultData(frame.ptr());
        callback(frame);
    }
}

// #*#*#*#*#*#*#*#*#*#*#*#*#*#*#*

class FaceFinderPainter::Impl
{
public:
    Impl()
    {
        m_tic = std::chrono::high_resolution_clock::now();

        m_positions = {
            { 0.1f, 0.1f },
            { 0.1f, 0.9f },
            { 0.9f, 0.1f },
            { 0.9f, 0.9f }
        };
    }
    ~Impl() {}

    std::chrono::high_resolution_clock::time_point m_tic;

    int m_index;
    std::vector<cv::Point2f> m_positions;

    bool m_showMotionAxes = true;
};

FaceFinderPainter::FaceFinderPainter(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext)
    : FaceFinder(factory, settings, glContext)
{
    m_pImpl = drishti::core::make_unique<Impl>();
    m_drawIris = true;
}

std::unique_ptr<FaceFinderPainter>
FaceFinderPainter::create(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext)
{
    auto finder = drishti::core::make_unique<FaceFinderPainter>(factory, settings, glContext);
    finder->initialize();
    return finder;
}

FaceFinderPainter::~FaceFinderPainter()
{
}

void FaceFinderPainter::setEffectKind(EffectKind kind)
{
    m_effect = kind;
}

auto FaceFinderPainter::getEffectKind() const -> EffectKind
{
    return m_effect;
}

void FaceFinderPainter::setShowMotionAxes(bool value)
{
    m_pImpl->m_showMotionAxes = value;
}

bool FaceFinderPainter::getShowMotionAxes() const
{
    return m_pImpl->m_showMotionAxes;
}

void FaceFinderPainter::setShowDetectionScales(bool value)
{
    m_painter->setShowDetectionScales(value);
}

bool FaceFinderPainter::getShowDetectionScales() const
{
    return m_painter->getShowDetectionScales();
}

// Get output pixels via callback (zero copy where possible):
void FaceFinderPainter::getOutputPixels(FrameDelegate& callback)
{
    getImage(*m_rotater, callback);
}

void FaceFinderPainter::setLetterboxHeight(float height)
{
    m_painter->setLetterboxHeight(height);
}

void FaceFinderPainter::init(const cv::Size& inputSize)
{
    m_inputSize = inputSize;
    FaceFinder::init(inputSize);
}

void FaceFinderPainter::initPainter(const cv::Size& inputSizeUp)
{
    m_inputSizeUp = inputSizeUp;
    FaceFinder::initPainter(inputSizeUp);

    // ### Painter ###
    m_outputOrientation = ::ogles_gpgpu::degreesToOrientation(360 - impl->outputOrientation);

    m_rotater = std::make_shared<ogles_gpgpu::TransformProc>();
    m_rotater->setOutputRenderOrientation(m_outputOrientation);

    m_painter = std::make_shared<ogles_gpgpu::FacePainter>(0);
    m_painter->setInterpolation(ogles_gpgpu::TransformProc::BILINEAR);

    if (impl->logger)
    {
        m_painter->setLogger(impl->logger);
    }

    { // Project detection sizes to full resolution image:
        auto winSize = impl->detector->getWindowSize();
        
        // Some detectors may expect column major storage,
        // so we transpose the detector windows dimensions here.
        if(!impl->detector->getIsRowMajor())
        {
            std::swap(winSize.width, winSize.height);
        }
        
        for (const auto& size : impl->pyramidSizes)
        {
            ogles_gpgpu::LineDrawing drawing;
            drawing.color = { 255, 0, 0 };

            const float wl0 = static_cast<float>(winSize.width * inputSizeUp.width) / size.width;
            const float hl0 = static_cast<float>(winSize.height * inputSizeUp.height) / size.height;
            // clang-format off
            drawing.contours =
            {{
                 { 0.f, 0.f },
                 { wl0, 0.f },
                 { wl0, hl0 },
                 { 0.f, hl0 }
            }};
            // clang-format on
            m_painter->getPermanentLineDrawings().push_back(drawing);
        }
    }

#if DRISHTI_HCI_FACE_FINDER_PAINTER_SHOW_CIRCLE
    m_circle = std::make_shared<ogles_gpgpu::CircleProc>();
    m_painter->add(m_circle.get());
    m_circle->add(m_rotater.get());
#else
    m_painter->add(m_rotater.get());
#endif

    m_painter->prepare(inputSizeUp.width, inputSizeUp.height, GL_RGBA);
}

template <typename Container>
void cat(const Container& a, const Container& b, Container& c)
{
    c.reserve(a.size() + b.size() + c.size());
    std::copy(a.begin(), a.end(), std::back_inserter(c));
    std::copy(b.begin(), b.end(), std::back_inserter(c));
}

GLuint FaceFinderPainter::filter(const ScenePrimitives& scene, GLuint inputTexture)
{
    // clang-format on
    MethodLog timeSummary(DRISHTI_LOCATION_SIMPLE);
    core::ScopeTimeLogger paintLogger = [&](double ts) {
        impl->logger->info("TIMING:{}={};{}", timeSummary.name, ts, timeSummary.ss.str());
    };
// clang-format off
    
#if DRISHTI_HCI_FACE_FINDER_PAINTER_SHOW_CIRCLE
    {
        const auto toc = std::chrono::high_resolution_clock::now();
        const double elapsed = std::chrono::duration<double>(toc - m_pImpl->m_tic).count();
        const int seconds = int(elapsed);
        int index = seconds % m_pImpl->m_positions.size();
        const auto& position = m_pImpl->m_positions[index];
        const float scale = std::cos(elapsed * 4.f) * 0.01;
        m_circle->setRadius(0.05f + scale);
        m_circle->setCenter({ position.x, position.y });
    }
#endif
    
    // Convert objects to line drawings
    m_painter->getLineDrawings().clear();
    
    // Always set motion axes:
    if (m_pImpl->m_showMotionAxes)
    {
        cv::Point3f motion = (m_pImpl->m_showMotionAxes ? (impl->faceMotion) : cv::Point3f(0.f, 0.f, 0.f));
        m_painter->setAxes(motion * 500.f);
    }
    
    // Note: scene.draw() muust be called prior to this point.  All drawing has been moved to the latency=1 cpu thread
    // for the previous frame to increase throughput.
    std::copy(scene.getDrawings().begin(), scene.getDrawings().end(), std::back_inserter(m_painter->getLineDrawings()));
    
    if (scene.faces().size())
    {
        core::ScopeTimeLogger faceTime = [&](double ts) { timeSummary.ss << "FACES=" << ts << ";"; };
        for (const auto& f : scene.faces())
        {
            m_painter->addFace(f);
        }
        
        // Note: These eye warps are provided wrt
        auto& eyeWarps = impl->eyeFilter->getEyeWarps();
        for (int i = 0; i < 2; i++)
        {
            eyeWarps[i].setContours(scene.m_eyeDrawings[i]);
        }
        
        { // Set the eye textures:
            m_painter->setEyeTexture(impl->eyeFilter->getOutputTexId(), impl->eyeFilter->getOutFrameSize(), eyeWarps);
            FeaturePoints eyePoints;
            
            if (false)
            {
                cat(impl->eyePoints[0], impl->eyePoints[1], eyePoints);
                m_painter->setEyePoints(eyePoints);
            }
        }
        
        if (impl->doIris && m_drawIris)
        {
            //Draw the normalized iris (polar coordinates):
            for (int i = 0; i < 2; i++)
            {
                m_painter->setIrisTexture(i, impl->ellipsoPolar[i]->getOutputTexId(), impl->ellipsoPolar[i]->getOutFrameSize());
            }
        }
    }
    else if (scene.objects().size())
    {
        rectanglesToDrawings(scene.objects() * impl->ACFScale, m_painter->getLineDrawings());
    }

    if (impl->doBlobs)
    {
        m_painter->setBlobTexture(impl->blobFilter->getOutputTexId(), impl->blobFilter->getOutFrameSize());
    }
    
    if (impl->doEyeFlow)
    {
        m_painter->setEyeFlow(impl->eyeFlowField);
        m_painter->setEyeMotion(impl->eyeMotion);
    }
    
    m_painter->setEyesWidthRatio(impl->renderEyesWidthRatio);
    
    {
        core::ScopeTimeLogger processTime = [&](double ts) { timeSummary.ss << "PROCESS=" << ts << ";"; };
        m_painter->process(inputTexture, 1, GL_TEXTURE_2D);
    }
    
    return m_rotater->getOutputTexId();
}

GLuint FaceFinderPainter::paint(const ScenePrimitives& scene, GLuint inputTexture)
{
    m_painter->setBrightness(impl->brightness);

    // Here we can choose one of several display layouts or effects:
    switch(m_effect)
    {
        case kStabilize : return stabilize(inputTexture, m_inputSizeUp, scene.faces().size() ? scene.faces()[0] : face::FaceModel());
        case kWireframes :
        default : return filter(scene, inputTexture);
    }
}

DRISHTI_HCI_NAMESPACE_END
