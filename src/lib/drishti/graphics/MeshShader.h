/*! -*-c++-*-
  @file   MeshShader.h
  @author David Hirvonen
  @brief Declaration of ogles_gpgpu shader for warping a face mesh.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_MeshShader_h__
#define __drishti_graphics_MeshShader_h__

#include "drishti/graphics/GLTexture.h"

#include <ogles_gpgpu/common/proc/base/filterprocbase.h>
#include <opencv2/core.hpp>
#include <glm/glm.hpp>

#include <memory>

BEGIN_OGLES_GPGPU

class MeshShader
{
public:
    using VertexBuffer = std::vector<glm::vec4>;
    using CoordBuffer = std::vector<glm::vec2>;

    MeshShader(const cv::Mat& iso, const VertexBuffer& vertices, const CoordBuffer& coords);

    static const char* getProcName();

    void setModelViewProjection(const glm::mat4& mvp);

    void draw(int outFrameW, int outFrameH);

protected:
    std::shared_ptr<Shader> shader;

    static const char* vshaderMeshSrc;
    static const char* fshaderMeshSrc;

    GLTexture texture;
    GLuint texUnit;
    GLuint texTarget;

    GLint shParamAPos;
    GLint shParamATexCoord;
    GLint shParamUInputTex;
    GLint shParamUMVP;

    VertexBuffer vertices;
    CoordBuffer coords;

    glm::mat4 MVP;
};

END_OGLES_GPGPU

#endif // __drishti_graphics_MeshShader_h__
