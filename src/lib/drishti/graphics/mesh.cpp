/*! -*-c++-*-
  @file   MeshProc
  @author David Hirvonen
  @brief Implementation of ogles_gpgpu shader for drawing lines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/mesh.h"

BEGIN_OGLES_GPGPU

// clang-format off
const char* MeshProc::fshaderPixelSrc = 
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision mediump float;)
#endif
OG_TO_STR(
varying vec2 vTexCoord;
uniform sampler2D uInputTex;
void main() {
    gl_FragColor = texture2D(uInputTex, vTexCoord);
});
// clang-format on

MeshProc::MeshProc(const drishti::graphics::MeshTex& mesh, const cv::Mat& iso, bool doWire)
{
    if (doWire)
    {
        drishti::graphics::MeshTex::VertexBuffer lines;
        mesh.getWireMeshSegments(lines);
        lineShader = std::make_shared<LineShader>(lines);
    }

    if (!iso.empty())
    {
        drishti::graphics::MeshTex::VertexBuffer vertices;
        drishti::graphics::MeshTex::CoordBuffer coords;
        mesh.getTriangleList(vertices, coords);
        meshShader = std::make_shared<MeshShader>(iso, vertices, coords);
    }
}

void MeshProc::setModelViewProjection(const glm::mat4& mvp)
{
    if (meshShader)
    {
        meshShader->setModelViewProjection(mvp);
    }
    if (lineShader)
    {
        lineShader->setModelViewProjection(mvp);
    }
}

void MeshProc::filterRenderCleanup()
{
    if (meshShader)
    {
        if (background.a)
        {
            glClearColor(background.r, background.g, background.b, background.a);
            glClear(GL_COLOR_BUFFER_BIT);
        }
        meshShader->draw(outFrameW, outFrameH);
    }
    if (lineShader)
    {
        lineShader->draw(outFrameW, outFrameH);
    }
    FilterProcBase::filterRenderCleanup();
}

END_OGLES_GPGPU
