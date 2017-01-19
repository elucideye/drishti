/*!
  @file  FacePainter.cpp
  @author David Hirvonen
  @brief Simple interface to draw faces and other primitives from the scene.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/FacePainter.h"
#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/geometry/motion.h"

#include <opencv2/imgproc.hpp>

#define DO_COLOR 1

using namespace std;
using namespace ogles_gpgpu;

struct DrawingSpec
{
    DrawingSpec(int type) : type(type) {}
    int type;
    std::vector<cv::Vec3f> colors;
    std::vector<cv::Point2f> points;
};

// =====

// *INDENT-OFF*
const char *FacePainter::fshaderLetterBoxSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
precision mediump float;
#endif

 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;

 uniform float colorWeight;
 uniform vec3 color;

void main()
{
    vec4 pixel = vec4(texture2D(uInputTex, vTexCoord).rgba);
    vec3 mixed = mix(pixel.rgb, color, colorWeight * abs(vTexCoord.y-0.5));
    gl_FragColor = vec4(mixed, 1.0);
}
);
// *INDENT-ON*

// ===== constant color shader ======

// *INDENT-OFF*
const char * FacePainter::vshaderColorSrc = OG_TO_STR
(
 uniform mat4 modelViewProjMatrix;
 void main()
 {
    color = vertexColor;
    gl_Position = modelViewProjMatrix * position;
 });
// *INDENT-ON*

// *INDENT-OFF*
const char * FacePainter::fshaderColorSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision highp float;
#endif
 uniform vec3 lineColor;
 varying color;
 void main()
 {
     gl_FragColor = vec4(color, 1.0);
 });
// *INDENT-ON*

// ===== varying color shader ======

// *INDENT-OFF*
const char * FacePainter::vshaderColorVaryingSrc = OG_TO_STR
(
 attribute vec4 position;
 attribute vec3 color;
 uniform mat4 modelViewProjMatrix;

 varying vec3 lineColor;
 void main()
 {
     lineColor = color;
     gl_Position = modelViewProjMatrix * position;
 });
// *INDENT-ON*

// *INDENT-OFF*
const char * FacePainter::fshaderColorVaryingSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision highp float;
#endif
 varying vec3 lineColor;
 void main()
 {
     gl_FragColor = vec4(lineColor, 1.0);
 });
// *INDENT-ON*

void FacePainter::getUniforms()
{
    TransformProc::getUniforms();

    m_drawShParamUMVP = m_draw->getParam(UNIF, "modelViewProjMatrix");
    
#if DO_COLOR
    m_drawShParamULineColor = 0;
#else
    m_drawShParamULineColor = m_draw->getParam(UNIF, "lineColor");
#endif

    // color overlays
    m_colorRGB = {0.0, 0.0, 1.0};
    m_colorWeight = 1.f;
    m_colorShParamRGB = shader->getParam(UNIF, "color");
    m_colorShParamUWeight = shader->getParam(UNIF, "colorWeight");
}

// #### Begin ####

FacePainter::FacePainter(int outputOrientation)
    : m_outputOrientation(outputOrientation)
{
    assert(m_outputOrientation == 0);

    setInterpolation(ogles_gpgpu::TransformProc::BILINEAR); // faster

    m_draw = std::make_shared<Shader>();
#if DO_COLOR
    bool compiled = m_draw->buildFromSrc(vshaderColorVaryingSrc, fshaderColorVaryingSrc);
    m_drawShParamAColor = m_draw->getParam(ATTR, "color");
#else
    bool compiled = m_draw->buildFromSrc(vshaderColorSrc, fshaderColorSrc);
    m_drawShParamULineColor = m_draw->getParam(UNIF, "lineColor");
#endif
    assert(compiled);
    m_drawShParamAPosition = m_draw->getParam(ATTR, "position");
    m_drawShParamUMVP = m_draw->getParam(UNIF, "modelViewProjMatrix");
}

void FacePainter::setOutputSize(float scaleFactor)
{
    /* make this a noop */
}
void FacePainter::setOutputRenderOrientation(RenderOrientation o)
{
    assert(false);
}

void FacePainter::clear()
{
    m_drawings.clear();
    m_faces.clear();
}

cv::Matx33f FacePainter::uprightImageToTexture()
{
    cv::Size2f size(inFrameW, inFrameH);

    // Transform to rectangle defined by (-1,-1) (+1,+1)
    const cv::Matx33f T = transformation::translate(-size.width/2, -size.height/2);
    const cv::Matx33f S = transformation::scale(2.0/size.width,2.0/size.height);
    cv::Matx33f H = S * T;

    return H;
}

void FacePainter::renderDrawings()
{
    DrawingSpec lines(GL_LINES);
    
    std::copy(m_permanentDrawings.begin(), m_permanentDrawings.end(), std::back_inserter(m_drawings));

    glLineWidth(3.0);
    for(const auto &e : m_drawings)
    {
        if(e.strip == true)
        {
            // Convert line strips to line segments for ios (no glMultiDrawArrays())
            for(auto &c : e.contours)
            {
                for(int i = 1; i < c.size(); i++)
                {
                    lines.points.emplace_back(c[i-1]);
                    lines.points.emplace_back(c[i+0]);
                    lines.colors.emplace_back(e.color);
                    lines.colors.emplace_back(e.color);
                }
            }
        }
        else
        {
            for(const auto &c : e.contours)
            {
                for(int i = 0; i < c.size(); i++)
                {
                    lines.points.emplace_back(c[i]);
                    lines.colors.emplace_back(e.color);
                }
            }
        }
    }

    m_draw->use();

    // Compensate for rotation:
    const cv::Matx33f H = uprightImageToTexture();
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(H.t(), MVPt);

    glUniformMatrix4fv(m_drawShParamUMVP, 1, 0, (GLfloat *)&MVPt(0,0));
    Tools::checkGLErr(getProcName(), "render drawings");

    glViewport(0, 0, outFrameW, outFrameH);
    Tools::checkGLErr(getProcName(), "glViewport()");

    glVertexAttribPointer(m_drawShParamAColor, 3, GL_FLOAT, 0, 0, &lines.colors[0]);
    Tools::checkGLErr(getProcName(), "glVertexAttribPointer()");

    glVertexAttribPointer(m_drawShParamAPosition, 2, GL_FLOAT, 0, 0, &lines.points[0]);
    Tools::checkGLErr(getProcName(), "glVertexAttribPointer()");

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.points.size()));
    Tools::checkGLErr(getProcName(), "glDrawArrays()");
}


void FacePainter::FacePainter::setUniforms()
{
    TransformProc::setUniforms();
}

int FacePainter::FacePainter::render(int position)
{
    OG_LOGINF(getProcName(), "input tex %d, target %d, framebuffer of size %dx%d", texId, texTarget, outFrameW, outFrameH);
    filterRenderPrepare();
    Tools::checkGLErr(getProcName(), "render prepare");

    setUniforms();

    if(m_flashInfo.texId >= 0)
    {
        const uint64_t frameCount = (m_frameIndex++ % 30);
        if(frameCount < 4)
        {
            // alternate the color on each frame: { 012345 }
            const int on = !(frameCount % 2);
            const int channelIndex = ((frameCount / 2) % 3);
            const float weight = 1.0f; // 2.0 * float(on);

            m_colorRGB = Vec3f(0.f, 0.f, 0.f);
            m_colorRGB.data[channelIndex] =  1.0;
            m_colorRGB.data[channelIndex] = float(on);

            glUniform3fv(m_colorShParamRGB, 1, &m_colorRGB.data[0]);
            glUniform1f(m_colorShParamUWeight, weight);
        }
    }

    filterRenderSetCoords();
    Tools::checkGLErr(getProcName(), "render set coords");

    // Draw the frame, line drawings and normalized face/eyes
    filterRenderDraw();
    renderDrawings();
    Tools::checkGLErr(getProcName(), "render draw");

    filterRenderCleanup();
    Tools::checkGLErr(getProcName(), "render cleanup");

    if(m_eyesInfo.texId >= 0)
    {
        renderTex(m_eyesInfo);
    }
    if(m_flowInfo.texId >= 0)
    {
        renderTex(m_flowInfo);
    }
    if(m_flashInfo.texId >= 0)
    {
        renderTex(m_flashInfo);
    }
    for(int i = 0; i < 2; i++)
    {
        if(m_irisInfo[i].texId >= 0)
        {
            renderTex(m_irisInfo[i]);
        }
    }

    clear();

    return 0;
}

int FacePainter::init(int inW, int inH, unsigned int order, bool prepareForExternalInput)
{
    return FilterProcBase::init(inW, inH, order, prepareForExternalInput);
}

//0, 0,
//1, 0,
//0, 1,
//1, 1

template <typename T>
std::vector<cv::Point_<T>> getCorners(const cv::Rect_<T> &roi)
{
    cv::Point_<T> tl = roi.tl(), br = roi.br(), tr(br.x, tl.y), bl(tl.x, br.y);
    return std::vector<cv::Point_<T>> { tl, tr, bl, br };
}

// =========================
// === Faces================
// =========================

void FacePainter::annotateEye(const cv::Rect &dstRoiPix, const cv::Matx33f &Heye, const DRISHTI_EYE::EyeModel &eye)
{
    auto contours = eye.getContours();

    DrawingSpec lines(0);
    for(auto &c : contours)
    {
        for(int i = 1; i < c.size(); i++)
        {
            lines.points.emplace_back(c[i-1]);
            lines.points.emplace_back(c[i+0]);

            lines.colors.emplace_back(0.0,1.0,0.0);
            lines.colors.emplace_back(0.0,1.0,0.0);
        }
    }

    glLineWidth(4.0);
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(Heye.t(), MVPt);

    glUniformMatrix4fv(m_drawShParamUMVP, 1, 0, (GLfloat *)&MVPt(0,0));
    Tools::checkGLErr(getProcName(), "FacePainter::renderEye() : glUniformMatrix4fv()");

    glVertexAttribPointer(m_drawShParamAColor, 3, GL_FLOAT, 0, 0, &lines.colors[0]);
    Tools::checkGLErr(getProcName(), "FacePainter::annotateEye() : glVertexAttribPointer()");

    glVertexAttribPointer(m_drawShParamAPosition, 2, GL_FLOAT, 0, 0, &lines.points[0]);
    Tools::checkGLErr(getProcName(), "FacePainter::annotateEye() : glVertexAttribPointer()");

    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(lines.points.size()));
    Tools::checkGLErr(getProcName(), "FacePainter::annotateEye() : glDrawArrays()");
}

// roi: eye roi in output frame
// H: src pixels => dest texels [-1 ... +1]
// N:

void FacePainter::renderEye(const cv::Rect &dstRoiPix, const cv::Matx33f &Heye, const DRISHTI_EYE::EyeModel &eye)
{
    // Limit warping to desired eye crop region
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(Heye.t(), MVPt);
    glScissor(dstRoiPix.x, dstRoiPix.y, dstRoiPix.width, dstRoiPix.height);

    glUniformMatrix4fv(shParamUTransform, 1, 0, (GLfloat *)&MVPt(0,0));
    Tools::checkGLErr(getProcName(), "FacePainter::renderEye() : glUniformMatrix4fv()");

    glDrawArrays(GL_TRIANGLE_STRIP, 0, OGLES_GPGPU_QUAD_VERTICES);
    Tools::checkGLErr(getProcName(), "FacePainter::renderEye() : glDrawArrays()");
}

// Layout criteria:
// 3x2 aspect ratio (centered)
// both left and right eyes
// don't exceed native specified width

std::array<drishti::eye::EyeWarp, 2> FacePainter::renderEyes(const drishti::face::FaceModel &face)
{
    drishti::face::FaceStabilizer stabilizer({inFrameW, inFrameH});
    std::array<drishti::eye::EyeWarp, 2> cropInfo = stabilizer.renderEyes(face, {inFrameW, inFrameH});

    for(int i = 0; i < 2; i++)
    {
        renderEye(cropInfo[i].roi, cropInfo[i].H, (i == 0) ? *face.eyeFullL : *face.eyeFullR);
    }
    return cropInfo;
}

void FacePainter::renderFaces()
{
    if(m_faces.size())
    {
        const cv::Size screenSize(outFrameW, outFrameH);

        // Make sure the main texture is bound:
        glActiveTexture(GL_TEXTURE0 + texUnit);
        glBindTexture(GL_TEXTURE_2D, texId);
        Tools::checkGLErr(getProcName(), "FacePainter::renderFaces() : glBindTexture()");

        // Set vertices so transformations can be performed in pixel coordinates:
        const std::vector<cv::Point2f> vertices = getCorners<float>(cv::Rect({0,0}, screenSize)); // tl, tr, bl, br
        glEnableVertexAttribArray(shParamAPos);
        glVertexAttribPointer(shParamAPos, 2, GL_FLOAT, GL_FALSE, 0, & vertices[0]);
        Tools::checkGLErr(getProcName(), "FacePainter::renderFaces() : glVertexAttribPointer()");
    }
}

// =====================
// === Eyes ============
// =====================

void FacePainter::annotateEyes()
{
    m_draw->use();
    Tools::checkGLErr(getProcName(), "m_draw->use()");

    glEnable(GL_SCISSOR_TEST);
    glScissor(m_eyesInfo.roi.x, m_eyesInfo.roi.y, m_eyesInfo.roi.width, m_eyesInfo.roi.height);
    for(const auto &eye : m_eyes)
    {
        annotateEye(eye.roi, eye.H, eye.eye);
    }
    glDisable(GL_SCISSOR_TEST);
}

// ################### UTILITY ############################

// Implement all utilty texture drawing in terms of these:
void FacePainter::renderTex(DisplayTexture &texInfo)
{
    OG_LOGINF(getProcName(), "input tex %d, target %d, framebuffer of size %dx%d", texId, texTarget, outFrameW, outFrameH);

    filterRenderPrepareTex(texInfo);
    Tools::checkGLErr(getProcName(), "render prepare");

    glUniform1f(m_colorShParamUWeight, 0.f);
    Tools::checkGLErr(getProcName(), "setUniforms");

    filterRenderSetCoordsTex(texInfo);
    Tools::checkGLErr(getProcName(), "render set coords");

    filterRenderDraw();
    Tools::checkGLErr(getProcName(), "render draw");

    texInfo.annotate();

    filterRenderCleanup();
    Tools::checkGLErr(getProcName(), "render cleanup");
}

void FacePainter::filterRenderPrepareTex(DisplayTexture &texInfo)
{
    // Use the same shader as the normal Eyes:
    shader->use();
    Tools::checkGLErr(getProcName(), "shader->use()");

    glViewport(texInfo.roi.x, texInfo.roi.y, texInfo.roi.width, texInfo.roi.height);
    Tools::checkGLErr(getProcName(), "glViewport()");

    assert(texTarget == GL_TEXTURE_2D);

    // set input texture
    glActiveTexture(GL_TEXTURE0 + texUnit);
    Tools::checkGLErr(getProcName(), "glActiveTexture()");

    glBindTexture(texTarget, texInfo.texId); // bind input texture
    Tools::checkGLErr(getProcName(), "glBindTexture()");

    // set common uniforms
    glUniform1i(shParamUInputTex, texUnit);
    Tools::checkGLErr(getProcName(), "glUniform1i()");
}

void FacePainter::filterRenderSetCoordsTex(DisplayTexture &texInfo)
{
    // render to FBO
    if (fbo)
    {
        fbo->bind();
    }

    // set geometry
    glEnableVertexAttribArray(shParamAPos);
    glVertexAttribPointer(shParamAPos, OGLES_GPGPU_QUAD_COORDS_PER_VERTEX, GL_FLOAT, GL_FALSE, 0, &ProcBase::quadVertices[0]);

    glVertexAttribPointer(shParamATexCoord, OGLES_GPGPU_QUAD_TEXCOORDS_PER_VERTEX, GL_FLOAT, GL_FALSE, 0, &ProcBase::quadTexCoordsStd[0]);
    glEnableVertexAttribArray(shParamATexCoord);
}
