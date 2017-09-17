/*! -*-c++-*-
  @file   LineShader.h
  @author David Hirvonen
  @brief Declaration of ogles_gpgpu shader for drawing lines.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_LineShader_h__
#define __drishti_graphics_LineShader_h__

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"
#include <glm/glm.hpp>

#include <array>
#include <memory>

BEGIN_OGLES_GPGPU

// Ideally we would use GL_TRIANGLES + glPolygonMode(GL_FRONT_AND_BACK,GL_LINE);
// However, this isn't supported in OpenGL ES
// https://stackoverflow.com/a/29583687
// https://stackoverflow.com/a/4063084

class LineShader
{
public:
    using VertexBuffer = std::vector<glm::vec4>;

    LineShader(const VertexBuffer& vertices);

    static const char* getProcName();

    void draw(int outFrameW, int outFrameH);

    void setModelViewProjection(const glm::mat4& mvp);

protected:
    std::shared_ptr<Shader> shader;

    static const char* vshaderColorSrc;
    static const char* fshaderColorSrc;

    GLint shParamUColor;
    GLint shParamUMVP;
    GLint shParamAPosition;

    std::vector<glm::vec4> points;
    std::array<float, 3> color = { { 1.f, 1.f, 1.f } };
    glm::mat4 MVP;
};

END_OGLES_GPGPU

#endif // __drishti_graphics_LineShader_h__
