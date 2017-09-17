/*! -*-c++-*-
  @file   EllipsoPolarWarp.cpp
  @author David Hirvonen
  @brief  Implementation of class for OpenGL ellipso-polar warping shader (via GL_TRIANGLE_STRIPS).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/gpu/EllipsoPolarWarp.h"

#include "drishti/eye/IrisNormalizer.h"

#include "drishti/geometry/motion.h"

#include <array>

BEGIN_OGLES_GPGPU

/*
 * Input eye models shall be in pixel coordinates associated with the input texture/FBO
 */

EllipsoPolarWarp::EllipsoPolarWarp()
{
}

void EllipsoPolarWarp::renderIris(const DRISHTI_EYE::EyeModel& eye)
{
    m_eye = m_eyeDelegate(); // get updated eye models:

    DRISHTI_EYE::IrisNormalizer::Rays rayPixels, rayTexels;
    DRISHTI_EYE::IrisNormalizer normalizer;

    // The ogles_gpgpu filter size is used to define the normalized iris dimensions:
    ogles_gpgpu::Size2d size(getOutFrameW(), getOutFrameH());

    // We can use the provided rays as a GL_TRIANGLE_STRIP
    normalizer.createRays(eye, { size.width, size.height }, rayPixels, rayTexels, 0);

    m_texels.resize(rayTexels.size() * 2);
    m_pixels.resize(rayPixels.size() * 2);

    memcpy(&m_texels[0], &rayTexels[0], sizeof(rayTexels[0]) * rayTexels.size());
    memcpy(&m_pixels[0], &rayPixels[0], sizeof(rayPixels[0]) * rayPixels.size());

    {
        // Map destination points into clip space [(-1,-1)... (+1,+1)]
        cv::Matx33f N = transformation::translate(-1.f, -1.f) * transformation::scale(2.0, 2.0);
        for (int i = 0; i < m_texels.size(); i++)
        {
            cv::Point3f q = N * m_texels[i];
            m_texels[i] = { q.x, q.y };
        }
    }

    // ====================================
    // === virtual API rendering calls ====
    // ====================================

    filterRenderSetCoords();
    Tools::checkGLErr(getProcName(), "render set coords");

    filterRenderDraw();
    Tools::checkGLErr(getProcName(), "render draw");
}

void EllipsoPolarWarp::renderIrises()
{
    renderIris(m_eye.eye);
}

// Override this to avoid multiple calls to fbo->bind()
void EllipsoPolarWarp::filterRenderSetCoords()
{
    // set geometry
    glEnableVertexAttribArray(shParamAPos);

    // Destination point in clip space:
    glVertexAttribPointer(shParamAPos, 2, GL_FLOAT, GL_FALSE, 0, &m_texels[0]); // [(-1,-1)  (+1,+1)]

    // Source point in texture space:
    glVertexAttribPointer(shParamATexCoord, 2, GL_FLOAT, GL_FALSE, 0, &m_pixels[0]); // [(0,0)  (1,1)]
    glEnableVertexAttribArray(shParamATexCoord);
}

int EllipsoPolarWarp::render(int position)
{
    OG_LOGINF(getProcName(), "input tex %d, target %d, framebuffer of size %dx%d", texId, texTarget, outFrameW, outFrameH);

    filterRenderPrepare();
    Tools::checkGLErr(getProcName(), "render prepare");

    setUniforms();
    Tools::checkGLErr(getProcName(), "setUniforms");

    // render to FBO
    if (fbo)
    {
        fbo->bind();
    }

    renderIrises();

    filterRenderCleanup();
    Tools::checkGLErr(getProcName(), "render cleanup");

    return 0;
}

END_OGLES_GPGPU
