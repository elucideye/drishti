/*!
  @file  FacePainter.cpp
  @author David Hirvonen
  @brief Simple interface to draw faces and other primitives from the scene.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/FacePainter.h"
#include "drishti/hci/Scene.hpp"
#include "drishti/hci/gpu/GLPrinter.h" // import font
#include "drishti/face/gpu/FaceStabilizer.h"
#include "drishti/geometry/motion.h"
#include "drishti/geometry/ConicSection.h"
#include "drishti/geometry/Cylinder.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/timing.h"
#include "drishti/core/Logger.h"

#include <opencv2/imgproc.hpp>

#define DRISHIT_HCI_FACEPAINTER_DO_COLOR 1
#define DRISHTI_HCI_FACEPAINTER_COLOR_TINTING 1
#define DRISHTI_HCI_FACEPAINTER_SHOW_FLASH_INPUT 0

using namespace std;
using namespace ogles_gpgpu;

struct DrawingSpec
{
    DrawingSpec(int type)
        : type(type)
    {
    }
    int type;
    std::vector<cv::Vec3f> colors;
    std::vector<cv::Point2f> points;
};

// =====

// clang-format off
const char *FacePainter::fshaderLetterBoxSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
precision mediump float;
#endif

 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;

 uniform float height;
 uniform vec3 color;

void main()
{
    vec4 pixel = vec4(texture2D(uInputTex, vTexCoord).rgba);
    vec3 mixed = mix(pixel.rgb, color, step(height*0.5, abs(vTexCoord.y-0.5)));
    gl_FragColor = vec4(mixed, 1.0);
}
);
// clang-format on

// ===== constant color shader ======

// clang-format off
const char * FacePainter::vshaderColorSrc = OG_TO_STR
(
 uniform mat4 modelViewProjMatrix;
 void main()
 {
    color = vertexColor;
    gl_Position = modelViewProjMatrix * position;
 });
// clang-format on

// clang-format off
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
// clang-format on

// ===== varying color shader ======

// clang-format off
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
// clang-format on

// clang-format off
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
// clang-format on

FacePainter::~FacePainter()
{
}

void FacePainter::getUniforms()
{
    TransformProc::getUniforms();

    m_drawShParamUMVP = m_draw->getParam(UNIF, "modelViewProjMatrix");

#if DRISHIT_HCI_FACEPAINTER_DO_COLOR
    m_drawShParamULineColor = 0;
#else
    m_drawShParamULineColor = m_draw->getParam(UNIF, "lineColor");
#endif

    m_colorShParamRGB = shader->getParam(UNIF, "color");
    m_colorShLetterboxHeight = shader->getParam(UNIF, "height");
}

// #### Begin ####

FacePainter::FacePainter(int outputOrientation)
    : m_outputOrientation(outputOrientation)
    , m_colorRGB(0.f, 0.f, 1.f)
    , m_colorLetterboxHeight(0.5f)
{
    assert(m_outputOrientation == 0);

    setInterpolation(ogles_gpgpu::TransformProc::BILINEAR); // faster

    // Font rendering:
    m_printer = drishti::core::make_unique<GLPrinterShader>();

    m_draw = std::make_shared<Shader>();
#if DRISHIT_HCI_FACEPAINTER_DO_COLOR
    bool compiled = m_draw->buildFromSrc(vshaderColorVaryingSrc, fshaderColorVaryingSrc);
    m_drawShParamAColor = m_draw->getParam(ATTR, "color");
#else
    bool compiled = m_draw->buildFromSrc(vshaderColorSrc, fshaderColorSrc);
    m_drawShParamULineColor = m_draw->getParam(UNIF, "lineColor");
#endif
    assert(compiled);
    m_drawShParamAPosition = m_draw->getParam(ATTR, "position");
    m_drawShParamUMVP = m_draw->getParam(UNIF, "modelViewProjMatrix");

    m_eyeAttributes = { &m_eyePoints, 8.f /*override*/, { 1.0, 0.0, 1.0 } };
    m_eyeAttributes.flow = &m_eyeFlow;
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
    const cv::Matx33f T = transformation::translate(-size.width / 2, -size.height / 2);
    const cv::Matx33f S = transformation::scale(2.0 / size.width, 2.0 / size.height);
    cv::Matx33f H = S * T;

    return H;
}

static void addCross(DrawingSpec& lines, const cv::Point2f& point, const cv::Vec3f& color, float span)
{
    static const cv::Point2f dx(1.f, 0.f), dy(0.f, 1.f);

    lines.points.push_back(point - (dx * span));
    lines.points.push_back(point + (dx * span));
    lines.points.push_back(point - (dy * span));
    lines.points.push_back(point + (dy * span));

    lines.colors.push_back(color);
    lines.colors.push_back(color);
    lines.colors.push_back(color);
    lines.colors.push_back(color);
}

void FacePainter::renderDrawings()
{
    //const std::string tag = DRISHTI_LOCATION_SIMPLE;
    //drishti::core::ScopeTimeLogger renderLogger = [&](double ts) { m_logger->info("TIMING: {} = {}", tag, ts) };

    DrawingSpec lines(GL_LINES);

    if (m_showDetectionScales)
    {
        std::copy(m_permanentDrawings.begin(), m_permanentDrawings.end(), std::back_inserter(m_drawings));
    }

    glLineWidth(4.0);
    for (const auto& e : m_drawings)
    {
        if (e.strip == true)
        {
            // Convert line strips to line segments for ios (no glMultiDrawArrays())
            for (auto& c : e.contours)
            {
                for (int i = 1; i < c.size(); i++)
                {
                    lines.points.emplace_back(c[i - 1]);
                    lines.points.emplace_back(c[i + 0]);
                    lines.colors.emplace_back(e.color);
                    lines.colors.emplace_back(e.color);
                }
            }
        }
        else
        {
            for (const auto& c : e.contours)
            {
                for (int i = 0; i < c.size(); i++)
                {
                    lines.points.emplace_back(c[i]);
                    lines.colors.emplace_back(e.color);
                }
            }
        }
    }

    {
        cv::Point2f origin(outFrameW / 2, outFrameH / 2);
        float scale = cv::norm(origin);
        lines.points.emplace_back(origin);
        lines.points.emplace_back(origin + m_eyeMotion * scale);
        lines.colors.emplace_back(1.f, 0.f, 1.f);
        lines.colors.emplace_back(1.f, 0.f, 1.f);
    }

    if (m_faces.size())
    { // Show position of nearest face
        const auto& position = (*m_faces.front().eyesCenter);
        std::wstringstream wss;
        wss << position.z;

        m_printer->begin();
        const float scale = 10.f;
        const float sx = scale / static_cast<float>(outFrameW);
        const float sy = scale / static_cast<float>(outFrameH);
        m_printer->printAt(wss.str(), 0.0f, 0.5f, sx, sy);
        m_printer->end();

        if (m_gazePoints.size())
        {
            const cv::Point2f origin(outFrameW / 2, outFrameH / 2);
            addCross(lines, origin, { 1.f, 1.f, 0.f }, 1000.f);
            for (const auto& f : m_gazePoints)
            { // Project normalized gaze point to screen:
                const float radius = (outFrameW / 2);
                const float gain = radius * 2.f;
                const cv::Point2f gaze = (f.point * gain) + origin;
                addCross(lines, gaze, { 1.f, 1.f, 0.5f }, 200.f * f.radius);
            }
        }
    }

    m_draw->use();

    // Compensate for rotation:
    const cv::Matx33f H = uprightImageToTexture();
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(H.t(), MVPt);

    glUniformMatrix4fv(m_drawShParamUMVP, 1, 0, (GLfloat*)&MVPt(0, 0));
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

void FacePainter::setAxes(const cv::Point3f& axes)
{
    m_motion = axes;
}

// Camera matrix (4x4):
//
// | right_x up_x forward_x position_x |
// | right_y up_y forward_y position_y |
// | right_z up_z forward_z position_z |
// |   0     0      0          1       |

void FacePainter::renderAxes()
{
    cv::Point3f motion(-m_motion.x, m_motion.y, m_motion.z);
    m_axes = drishti::geometry::drawAxes(motion, 0.05f, 32, 0.125f);
    m_axesColors = m_axes;
    for (int i = 0; i < m_axesColors.size(); i++)
    {
        cv::Point3f color(float(i == 0), float(i == 1), float(i == 2));
        auto& axis = m_axesColors[i];
        std::fill(axis.begin(), axis.end(), color);
    }

    if (m_axes.size())
    {
        const float aspectRatio = static_cast<float>(outFrameW) / static_cast<float>(outFrameH);
        cv::Matx44f K = transformation::glPerspective(1000.f, aspectRatio, 0.f, 100.f);

        const float theta = M_PI;
        cv::Matx44f R = cv::Matx44f::eye();
        R(0, 0) = +std::cos(theta);
        R(0, 2) = -std::sin(theta);
        R(2, 0) = +std::sin(theta);
        R(2, 2) = +std::cos(theta);

        cv::Matx44f T = cv::Matx44f::eye();
        T(0, 3) = -4.0f;  // X
        T(1, 3) = +4.0f;  // Y
        T(2, 3) = -16.0f; // Z

        cv::Matx44f P = T * R;

        cv::Matx44f MVP, MVPt;
        MVP = K * P;
        MVPt = MVP.t();

        glLineWidth(1.f);

        m_draw->use();
        Tools::checkGLErr(getProcName(), "m_draw->use()");

        glUniformMatrix4fv(m_drawShParamUMVP, 1, 0, (GLfloat*)&MVPt(0, 0));
        Tools::checkGLErr(getProcName(), "render drawings");

        glViewport(0, 0, outFrameW, outFrameH);
        Tools::checkGLErr(getProcName(), "glViewport()");

        for (int i = m_axes.size() - 1; i >= 0; i--)
        {
            glVertexAttribPointer(m_drawShParamAColor, 3, GL_FLOAT, 0, 0, &m_axesColors[i].front().x);
            Tools::checkGLErr(getProcName(), "glVertexAttribPointer()");

            glVertexAttribPointer(m_drawShParamAPosition, 3, GL_FLOAT, 0, 0, &m_axes[i].front().x);
            Tools::checkGLErr(getProcName(), "glVertexAttribPointer()");

            glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(m_axes[i].size()));
            Tools::checkGLErr(getProcName(), "glDrawArrays()");
        }
    }
}

void FacePainter::renderGaze()
{
}

void FacePainter::FacePainter::setUniforms()
{
    TransformProc::setUniforms();
}

int FacePainter::FacePainter::render(int position)
{
    const std::string tag = DRISHTI_LOCATION_SIMPLE;
    drishti::core::ScopeTimeLogger renderLogger = [&](double ts) { m_logger->info("TIMING:{}={}", tag, ts); };

    OG_LOGINF(getProcName(), "input tex %d, target %d, framebuffer of size %dx%d", texId, texTarget, outFrameW, outFrameH);

    { // ... main render routine ...
        filterRenderPrepare();
        Tools::checkGLErr(getProcName(), "render prepare");
        setUniforms();

#if DRISHTI_HCI_FACEPAINTER_COLOR_TINTING
        if (m_flashInfo.texId >= 0)
        {
            m_colorRGB = Vec3f(0.f, 0.f, 0.f);
            m_colorRGB.data[0] = 1.f;
            m_colorRGB.data[1] = 1.f;
            m_colorRGB.data[2] = 1.f;

            glUniform3fv(m_colorShParamRGB, 1, &m_colorRGB.data[0]);
        }
#endif

        filterRenderSetCoords();
        Tools::checkGLErr(getProcName(), "render set coords");

        // Draw the frame, line drawings and normalized face/eyes
        filterRenderDraw();
        renderDrawings(); // 2d

        if (m_motion.dot(m_motion) > 0.f)
        {
            renderAxes(); // render w/ glPerspective (world coordinates)
        }

        Tools::checkGLErr(getProcName(), "render draw");

        filterRenderCleanup();
        Tools::checkGLErr(getProcName(), "render cleanup");
    }

    if (m_eyes.m_eyesInfo.texId >= 0)
    {
        renderTex(m_eyes.m_eyesInfo);
    }
    if (m_flowInfo.texId >= 0)
    {
        renderTex(m_flowInfo);
    }

#if DRISHTI_HCI_FACEPAINTER_SHOW_FLASH_INPUT
    if (m_flashInfo.texId >= 0)
    {
        renderTex(m_flashInfo);
    }
#endif

    for (int i = 0; i < 2; i++)
    {
        if (m_irisInfo[i].texId >= 0)
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
std::vector<cv::Point_<T>> getCorners(const cv::Rect_<T>& roi)
{
    cv::Point_<T> tl = roi.tl(), br = roi.br(), tr(br.x, tl.y), bl(tl.x, br.y);
    return std::vector<cv::Point_<T>>{ tl, tr, bl, br };
}

// =========================
// === Faces================
// =========================

static void
drawCrosses(const FacePainter::FeaturePoints& points, const cv::Vec3f& color, DrawingSpec& lines, float span)
{
    drishti::hci::LineDrawingVec crosses;
    drishti::hci::pointsToCircles(points, crosses, span);

    for (const auto& x : crosses)
    {
        for (const auto& c : x.contours)
        {
            for (const auto& p : c)
            {
                lines.points.emplace_back(p);
                lines.colors.emplace_back(color);
            }
        }
    }
}

static void
drawFlow(const FacePainter::FlowField& flow, const cv::Vec3f& color, DrawingSpec& lines, float span)
{
    drishti::hci::LineDrawingVec needles;

    for (const auto& f : flow)
    {
        lines.points.emplace_back(f[0], f[1]);
        lines.points.emplace_back(f[0] + f[2], f[1] + f[3]);
        lines.colors.emplace_back(color);
        lines.colors.emplace_back(color);
    }
}

/*
 * dstRoiPix : destination roi for texture in image coordinates
 * eye : eye model in full frame coordinates
 * Heye : transformation from full frame to
 */

void FacePainter::annotateEye(const drishti::eye::EyeWarp& eyeWarp, const cv::Size& size, const EyeAttributes& attributes)
{
    //const std::string tag = DRISHTI_LOCATION_SIMPLE;
    //drishti::core::ScopeTimeLogger paintLogger = [&](double ts) { m_logger->info("TIMING: {} = {}", tag, ts); }

    auto contours = eyeWarp.getContours(false); //!m_eyePoints.size());

    DrawingSpec lines(0);
    for (auto& c : contours)
    {
        for (int i = 1; i < c.size(); i++)
        {
            lines.points.emplace_back(c[i - 1]);
            lines.points.emplace_back(c[i + 0]);

            lines.colors.emplace_back(0.0, 1.0, 0.0);
            lines.colors.emplace_back(0.0, 1.0, 0.0);
        }
    }

    if (attributes.points && attributes.points->size())
    {
        drawCrosses(*attributes.points, attributes.color, lines, attributes.scale);
    }

    if (attributes.flow && attributes.flow->size())
    {
        drawFlow(*attributes.flow, attributes.color, lines, 100.f);
    }

    glLineWidth(2.0);
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(eyeWarp.H.t(), MVPt);

    glUniformMatrix4fv(m_drawShParamUMVP, 1, 0, (GLfloat*)&MVPt(0, 0));
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

void FacePainter::renderEye(const cv::Rect& dstRoiPix, const cv::Matx33f& Heye, const DRISHTI_EYE::EyeModel& eye)
{
    // Limit warping to desired eye crop region
    cv::Matx44f MVPt;
    transformation::R3x3To4x4(Heye.t(), MVPt);
    glScissor(dstRoiPix.x, dstRoiPix.y, dstRoiPix.width, dstRoiPix.height);

    glUniformMatrix4fv(shParamUTransform, 1, 0, (GLfloat*)&MVPt(0, 0));
    Tools::checkGLErr(getProcName(), "FacePainter::renderEye() : glUniformMatrix4fv()");

    glDrawArrays(GL_TRIANGLE_STRIP, 0, OGLES_GPGPU_QUAD_VERTICES);
    Tools::checkGLErr(getProcName(), "FacePainter::renderEye() : glDrawArrays()");
}

// Render eye crops from full image texture:
//
// Layout criteria:
// 3x2 aspect ratio (centered)
// both left and right eyes
// don't exceed native specified width

std::array<drishti::eye::EyeWarp, 2> FacePainter::renderEyes(const drishti::face::FaceModel& face)
{
    drishti::face::FaceStabilizer stabilizer({ inFrameW, inFrameH });
    std::array<drishti::eye::EyeWarp, 2> cropInfo = stabilizer.renderEyes(face, { inFrameW, inFrameH });

    for (int i = 0; i < 2; i++)
    {
        renderEye(cropInfo[i].roi, cropInfo[i].H, (i == 0) ? *face.eyeFullL : *face.eyeFullR);
    }

    annotateEyes(m_eyes, m_eyeAttributes);

    return cropInfo;
}

void FacePainter::renderFaces()
{
    if (m_faces.size())
    {
        const cv::Size screenSize(outFrameW, outFrameH);

        // Make sure the main texture is bound:
        glActiveTexture(GL_TEXTURE0 + texUnit);
        glBindTexture(GL_TEXTURE_2D, texId);
        Tools::checkGLErr(getProcName(), "FacePainter::renderFaces() : glBindTexture()");

        // Set vertices so transformations can be performed in pixel coordinates:
        const std::vector<cv::Point2f> vertices = getCorners<float>(cv::Rect({ 0, 0 }, screenSize)); // tl, tr, bl, br
        glEnableVertexAttribArray(shParamAPos);
        glVertexAttribPointer(shParamAPos, 2, GL_FLOAT, GL_FALSE, 0, &vertices[0]);
        Tools::checkGLErr(getProcName(), "FacePainter::renderFaces() : glVertexAttribPointer()");
    }
}

// =====================
// === Eyes ============
// =====================

void FacePainter::annotateEyes(const EyePairInfo& eyes, const EyeAttributes& attributes)
{
    m_draw->use();
    Tools::checkGLErr(getProcName(), "m_draw->use()");

    glEnable(GL_SCISSOR_TEST);
    const auto& roi = eyes.m_eyesInfo.roi;
    glScissor(roi.x, roi.y, roi.width, roi.height);

    for (const auto& eye : eyes.m_eyes)
    {
        annotateEye(eye, { eyes.m_eyesInfo.size.width, eyes.m_eyesInfo.size.height }, attributes);
    }
    glDisable(GL_SCISSOR_TEST);
}

// ################### UTILITY ############################

// Implement all utilty texture drawing in terms of these:
void FacePainter::renderTex(DisplayTexture& texInfo)
{
    OG_LOGINF(getProcName(), "input tex %d, target %d, framebuffer of size %dx%d", texId, texTarget, outFrameW, outFrameH);

    filterRenderPrepareTex(texInfo);
    Tools::checkGLErr(getProcName(), "render prepare");

    glUniform1f(m_colorShLetterboxHeight, m_colorLetterboxHeight);
    Tools::checkGLErr(getProcName(), "setUniforms");

    filterRenderSetCoordsTex(texInfo);
    Tools::checkGLErr(getProcName(), "render set coords");

    filterRenderDraw();
    Tools::checkGLErr(getProcName(), "render draw");

    texInfo.annotate();

    filterRenderCleanup();
    Tools::checkGLErr(getProcName(), "render cleanup");
}

void FacePainter::filterRenderPrepareTex(DisplayTexture& texInfo)
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

void FacePainter::filterRenderSetCoordsTex(DisplayTexture& texInfo)
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
