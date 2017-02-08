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
#include "drishti/hci/gpu/FlashFilter.h"
#include "drishti/eye/gpu/EllipsoPolarWarp.h"
#include "drishti/core/drishti_operators.h"
#include "drishti/core/make_unique.h"
#include "drishti/geometry/motion.h"

#include "ogles_gpgpu/common/types.h"
#include "ogles_gpgpu/common/proc/transform.h"

#include <memory>

using drishti::face::operator*;
using drishti::core::operator*;

DRISHTI_HCI_NAMESPACE_BEGIN

// #*#*#*#*#*#*#*#*#*#*#*#*#*#*#*

class FaceFinderPainter::Impl
{
public:
    Impl() {}
    ~Impl() {}
};

FaceFinderPainter::FaceFinderPainter(FaceDetectorFactoryPtr &factory, Config &config, void *glContext)
    : FaceFinder(factory, config, glContext)
{
    m_pImpl = drishti::core::make_unique<Impl>();
    m_drawIris = true;
}

FaceFinderPainter::~FaceFinderPainter()
{
    
}

void FaceFinderPainter::init(const FrameInput &frame)
{
    FaceFinder::init(frame);
}

void FaceFinderPainter::initPainter(const cv::Size &inputSizeUp)
{
    FaceFinder::initPainter(inputSizeUp);
    
    // ### Painter ###
    ogles_gpgpu::RenderOrientation outputOrientation = ::ogles_gpgpu::degreesToOrientation(360 - m_outputOrientation);

    m_rotater = std::make_shared<ogles_gpgpu::TransformProc>();
    m_rotater->setOutputRenderOrientation(outputOrientation);

    m_painter = std::make_shared<ogles_gpgpu::FacePainter>(0);
    
    // Project detection sizes to full resolution image:
    const auto winSize = m_detector->getWindowSize();
    for(const auto &size : m_pyramidSizes)
    {
        ogles_gpgpu::LineDrawing drawing;
        drawing.color = {255,0,0};
        
        const float wl0 = static_cast<float> (winSize.width * inputSizeUp.width) / size.width;
        const float hl0 = static_cast<float> (winSize.height * inputSizeUp.height) / size.height;
        drawing.contours =
        {
            {
                {0.f, 0.f},
                {wl0, 0.f},
                {wl0, hl0},
                {0.f, hl0}
            }
        };
        m_painter->getPermanentLineDrawings().push_back(drawing);
    }
    
    m_painter->add(m_rotater.get());
    m_painter->prepare(inputSizeUp.width, inputSizeUp.height, GL_RGBA);
}

GLuint FaceFinderPainter::paint(const ScenePrimitives &scene, GLuint inputTexture)
{
    m_painter->setBrightness(m_brightness);
    //m_painter->setGazePoint(m_gazePoints);

    // Convert objects to line drawings
    m_painter->getLineDrawings().clear();

    if(scene.corners().size())
    {
        pointsToCrosses(scene.corners(), m_painter->getLineDrawings());
    }

    if(scene.faces().size())
    {
        facesToDrawings(scene.faces(), m_painter->getLineDrawings());
        for(const auto &f : scene.faces())
        {
            m_painter->addFace(f);
        }
        
        // Note: These eye warps are provided wrt
        const auto &eyeWarps = m_eyeFilter->getEyeWarps();
        const cv::Size filteredEyeSize(m_flasher->getOutFrameW(), m_flasher->getOutFrameH());

        m_painter->setEyeTextureA(m_eyeFilter->getOutputTexId(), m_eyeFilter->getOutFrameSize(), eyeWarps);
        FeaturePoints eyePointsSingle;
        std::copy(m_eyePointsSingle[0].begin(), m_eyePointsSingle[0].end(), std::back_inserter(eyePointsSingle));
        std::copy(m_eyePointsSingle[1].begin(), m_eyePointsSingle[1].end(), std::back_inserter(eyePointsSingle));
        m_painter->setEyePointsFromSingleImage(eyePointsSingle);
        
        if(m_doDifferenceEyesDisplay)
        {
            m_painter->setEyeTextureB(m_flasher->getOutputTexId(), m_flasher->getOutFrameSize(), eyeWarps);
            FeaturePoints eyePointsDifference;
            std::copy(m_eyePointsDifference[0].begin(), m_eyePointsDifference[0].end(), std::back_inserter(eyePointsDifference));
            std::copy(m_eyePointsDifference[1].begin(), m_eyePointsDifference[1].end(), std::back_inserter(eyePointsDifference));
            m_painter->setEyePointsFromDifferenceImage(eyePointsDifference);
        }

        if(m_doIris && m_drawIris)
        {
            //Draw the normalized iris (polar coordinates):
            for(int i = 0; i < 2; i++)
            {
                m_painter->setIrisTexture(i, m_ellipsoPolar[i]->getOutputTexId(), m_ellipsoPolar[i]->getOutFrameSize());
            }
        }
    }
    else if(scene.objects().size())
    {
        rectanglesToDrawings(scene.objects() * m_ACFScale, m_painter->getLineDrawings());
    }

    if(m_doFlow)
    {
        if(scene.flow().size())
        {
            flowToDrawings(scene.flow(), m_painter->getLineDrawings(), m_colors32FC3);
        }

        // Add the flow for debugging:
        auto *flow = m_acf->getFlowProc();
        m_painter->setFlowTexture(flow->getOutputTexId(), flow->getOutFrameSize());
    }

    if(m_doFlash)
    {
        m_painter->setFlashTexture(m_flasher->getOutputTexId(), m_flasher->getOutFrameSize());
    }

    m_painter->setAxes(m_faceMotion * 500.f);
    
    m_painter->process(inputTexture, 1, GL_TEXTURE_2D);
    
    return m_rotater->getOutputTexId();
}

DRISHTI_HCI_NAMESPACE_END
