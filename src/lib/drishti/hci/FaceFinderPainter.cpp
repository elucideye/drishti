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
#include "drishti/core/drishti_operators.h"

#include "ogles_gpgpu/common/types.h"
#include "ogles_gpgpu/common/proc/transform.h"

using drishti::face::operator*;
using drishti::core::operator*;

DRISHTI_HCI_NAMESPACE_BEGIN

FaceFinderPainter::FaceFinderPainter(FaceDetectorFactoryPtr &factory, Config &config, void *glContext)
    : FaceFinder(factory, config, glContext)
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
            m_eyeFilter->addFace(f);
        }

        // Configure eye enhancer
        m_eyeFilter->process(inputTexture, 1, GL_TEXTURE_2D);
        m_painter->setEyeTexture(m_eyeFilter->getOutputTexId(), m_eyeFilter->getOutFrameSize(), m_eyeFilter->getEyeWarps());

#if DRISHTI_FACEFILTER_DO_ELLIPSO_POLAR
        //Draw the polar warp:
        for(int i = 0; i < 2; i++)
        {
            m_painter->setIrisTexture(i, m_ellipsoPolar[i]->getOutputTexId(), m_ellipsoPolar[i]->getOutFrameSize());
        }
#endif
    }
    else if(scene.objects().size())
    {
        rectanglesToDrawings(scene.objects() * m_scale, m_painter->getLineDrawings());
    }

    if(m_doFlow)
    {
        if(scene.flow().size())
        {
            flowToDrawings(scene.flow(), m_painter->getLineDrawings(), m_colors32FC3);
        }

        // Add the flow for debugging:
        m_painter->setFlowTexture(m_acf->flow.getOutputTexId(), m_acf->flow.getOutFrameSize());
    }

    if(m_doFlash)
    {
        m_painter->setFlashTexture(m_flasher->last()->getOutputTexId(), m_flasher->last()->getOutFrameSize());
    }

    m_painter->process(inputTexture, 1, GL_TEXTURE_2D);
    
    return m_rotater->getOutputTexId();
}

DRISHTI_HCI_NAMESPACE_END

