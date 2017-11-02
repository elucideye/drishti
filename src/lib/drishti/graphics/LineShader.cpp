/*! -*-c++-*-
  @file   LineShader.cpp
  @author David Hirvonen
  @brief Implementation of ogles_gpgpu shader for drawing lines.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/LineShader.h"

BEGIN_OGLES_GPGPU

// clang-format off
const char * LineShader::vshaderColorSrc = OG_TO_STR
(
 attribute vec4 position;
 uniform mat4 modelViewProjMatrix;
 
 void main()
 {
     gl_Position = modelViewProjMatrix * position;
 });
// clang-format on

// clang-format off
const char * LineShader::fshaderColorSrc = 
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision highp float;)
#endif
OG_TO_STR(
 uniform vec3 lineColor;
 void main()
 {
     gl_FragColor = vec4(lineColor, 1.0);
 });
// clang-format on

LineShader::LineShader(const VertexBuffer& vertices)
    : points(vertices)
    , MVP(glm::mat4())
{
    // Compile utility line shader:
    shader = std::make_shared<Shader>();
    if (!shader->buildFromSrc(vshaderColorSrc, fshaderColorSrc))
    {
        throw std::runtime_error("LineShader: shader error");
    }
    shParamUColor = shader->getParam(UNIF, "lineColor");
    shParamUMVP = shader->getParam(UNIF, "modelViewProjMatrix");
    shParamAPosition = shader->getParam(ATTR, "position");
}

const char* LineShader::getProcName()
{
    return "LineShader";
}

void LineShader::setModelViewProjection(const glm::mat4& mvp)
{
    MVP = mvp;
}

void LineShader::draw(int outFrameW, int outFrameH)
{
    shader->use();
    glUniform3f(shParamUColor, color[0], color[1], color[2]);
    glUniformMatrix4fv(shParamUMVP, 1, 0, (GLfloat*)&MVP[0][0]);
    glViewport(0, 0, outFrameW, outFrameH);
    glVertexAttribPointer(shParamAPosition, 4, GL_FLOAT, 0, 0, &points.data()[0]);
    glDrawArrays(GL_LINES, 0, static_cast<GLsizei>(points.size() / 2));

    Tools::checkGLErr(getProcName(), "draw()");
};

END_OGLES_GPGPU
