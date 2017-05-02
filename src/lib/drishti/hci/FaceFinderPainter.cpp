/*!
  @file   drishti/hci/FaceFinderPainter.cpp
  @author David Hirvonen
  @brief  Face tracking class with OpenGL texture drawing

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/FaceFinderPainter.h"
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

#include "ogles_gpgpu/common/types.h"
#include "ogles_gpgpu/common/proc/transform.h"

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
};

FaceFinderPainter::FaceFinderPainter(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext)
    : FaceFinder(factory, settings, glContext)
{
    m_pImpl = drishti::core::make_unique<Impl>();
    m_drawIris = true;
}

std::unique_ptr<FaceFinderPainter> FaceFinderPainter::create(FaceDetectorFactoryPtr& factory, Settings& settings, void* glContext)
{
    auto finder = drishti::core::make_unique<FaceFinderPainter>(factory, settings, glContext);
    finder->initialize();
    return finder;
}

FaceFinderPainter::~FaceFinderPainter()
{
}

void FaceFinderPainter::init(const cv::Size& inputSize)
{
    FaceFinder::init(inputSize);
}

void FaceFinderPainter::initPainter(const cv::Size& inputSizeUp)
{
    FaceFinder::initPainter(inputSizeUp);

    // ### Painter ###
    ogles_gpgpu::RenderOrientation outputOrientation = ::ogles_gpgpu::degreesToOrientation(360 - m_outputOrientation);

    m_rotater = std::make_shared<ogles_gpgpu::TransformProc>();
    m_rotater->setOutputRenderOrientation(outputOrientation);

    m_painter = std::make_shared<ogles_gpgpu::FacePainter>(0);
    m_painter->setInterpolation(ogles_gpgpu::TransformProc::BILINEAR);

    if (m_logger)
    {
        m_painter->setLogger(m_logger);
    }

    // Project detection sizes to full resolution image:
    const auto winSize = m_detector->getWindowSize();
    for (const auto& size : m_pyramidSizes)
    {
        ogles_gpgpu::LineDrawing drawing;
        drawing.color = { 255, 0, 0 };

        const float wl0 = static_cast<float>(winSize.width * inputSizeUp.width) / size.width;
        const float hl0 = static_cast<float>(winSize.height * inputSizeUp.height) / size.height;
        drawing.contours = {
            { { 0.f, 0.f },
                { wl0, 0.f },
                { wl0, hl0 },
                { 0.f, hl0 } }
        };
        m_painter->getPermanentLineDrawings().push_back(drawing);
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

GLuint FaceFinderPainter::paint(const ScenePrimitives& scene, GLuint inputTexture)
{
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

    MethodLog timeSummary(DRISHTI_LOCATION_SIMPLE);
    core::ScopeTimeLogger paintLogger = [&](double ts) {
        m_logger->info() << "TIMING:" << timeSummary.name << "=" << ts << ";" << timeSummary.ss.str();
    };

    m_painter->setBrightness(m_brightness);

    // Convert objects to line drawings
    m_painter->getLineDrawings().clear();

    // Always set motion axes:
    m_painter->setAxes(m_faceMotion * 500.f);

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
        auto& eyeWarps = m_eyeFilter->getEyeWarps();
        for (int i = 0; i < 2; i++)
        {
            eyeWarps[i].setContours(scene.m_eyeDrawings[i]);
        }

        // Create eye contour models in parallel
        //auto result0 = m_threads->process([&] { eyeWarps[0].getContours(); });
        //auto result1 = m_threads->process([&] { eyeWarps[1].getContours(); });

        if (m_doIris && m_drawIris)
        {
            //Draw the normalized iris (polar coordinates):
            for (int i = 0; i < 2; i++)
            {
                m_painter->setIrisTexture(i, m_ellipsoPolar[i]->getOutputTexId(), m_ellipsoPolar[i]->getOutFrameSize());
            }
        }

        //result0.get();
        //result1.get();

        {
            m_painter->setEyeTexture(m_eyeFilter->getOutputTexId(), m_eyeFilter->getOutFrameSize(), eyeWarps);
            FeaturePoints eyePoints;
            cat(m_eyePoints[0], m_eyePoints[1], eyePoints);
            m_painter->setEyePoints(eyePoints);
        }
    }
    else if (scene.objects().size())
    {
        rectanglesToDrawings(scene.objects() * m_ACFScale, m_painter->getLineDrawings());
    }

    if (m_doFlow)
    {
        core::ScopeTimeLogger flowTime = [&](double ts) { timeSummary.ss << "FLOW=" << ts << ";"; };

        if (scene.flow().size())
        {
            flowToDrawings(scene.flow(), m_painter->getLineDrawings(), m_colors32FC3);
        }

        // Add the flow for debugging:
        auto* flow = m_acf->getFlowProc();
        m_painter->setFlowTexture(flow->getOutputTexId(), flow->getOutFrameSize());
    }

    if (m_doFlash)
    {
        m_painter->setFlashTexture(m_flasher->getOutputTexId(), m_flasher->getOutFrameSize());
    }

    if (m_doEyeFlow)
    {
        m_painter->setEyeFlow(m_eyeFlowField);
        m_painter->setEyeMotion(m_eyeMotion);
    }

    {
        core::ScopeTimeLogger processTime = [&](double ts) { timeSummary.ss << "PROCESS=" << ts << ";"; };
        m_painter->process(inputTexture, 1, GL_TEXTURE_2D);
    }

    return m_rotater->getOutputTexId();
}

DRISHTI_HCI_NAMESPACE_END
