/*! -*-c++-*-
  @file   MeshShader.cpp
  @author David Hirvonen
  @brief Implementation of ogles_gpgpu shader for warping a face mesh.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/MeshShader.h"

// clang-format off
#ifdef ANDROID
#  define TEXTURE_FORMAT GL_RGBA
#else
#  define TEXTURE_FORMAT GL_BGRA
#endif
// clang-format on

BEGIN_OGLES_GPGPU

// clang-format off
const char * MeshShader::vshaderMeshSrc = OG_TO_STR
(
 attribute vec4 aPos;
 attribute vec2 aTexCoord;
 varying vec2 vTexCoord;
 uniform mat4 transformMatrix;
 void main()
 {
    gl_Position = transformMatrix * vec4(aPos.xyz, 1.0);
    vTexCoord = aTexCoord;
 }
 );
// clang-format on

// clang-format off
const char * MeshShader::fshaderMeshSrc =
#if defined(OGLES_GPGPU_OPENGLES)
 OG_TO_STR(precision mediump float;)
#endif
 OG_TO_STR(
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 void main()
 {
    gl_FragColor = vec4(texture2D(uInputTex, vTexCoord).rgba);
 }
);
// clang-format on

MeshShader::MeshShader(const cv::Mat& iso, const VertexBuffer& vertices, const CoordBuffer& coords)
    : texture(iso.cols, iso.rows, TEXTURE_FORMAT, const_cast<void*>(iso.ptr<void>()))
    , texUnit(1)
    , texTarget(GL_TEXTURE_2D)
    , vertices(vertices)
    , coords(coords)
{
    MVP = glm::mat4();

    // Compile utility shader:
    shader = std::make_shared<Shader>();
    if (!shader->buildFromSrc(vshaderMeshSrc, fshaderMeshSrc))
    {
        throw std::runtime_error("MeshShader: shader error");
    }

    shParamAPos = shader->getParam(ATTR, "aPos");
    shParamATexCoord = shader->getParam(ATTR, "aTexCoord");
    shParamUInputTex = shader->getParam(UNIF, "uInputTex");
    shParamUMVP = shader->getParam(UNIF, "transformMatrix");
}

const char* MeshShader::getProcName()
{
    return "MeshShader";
}

void MeshShader::setModelViewProjection(const glm::mat4& mvp)
{
    MVP = mvp;
}

void MeshShader::draw(int outFrameW, int outFrameH)
{
    shader->use();

    //glViewport(0, 0, outFrameW, outFrameH);

    assert(texTarget == GL_TEXTURE_2D);

    // set input texture
    glActiveTexture(GL_TEXTURE0 + texUnit);
    glBindTexture(texTarget, texture.texId); // bind input texture
    glUniform1i(shParamUInputTex, texUnit);
    glUniformMatrix4fv(shParamUMVP, 1, 0, (GLfloat*)&MVP[0][0]);
    glEnableVertexAttribArray(shParamAPos);
    glVertexAttribPointer(shParamAPos, 4, GL_FLOAT, GL_FALSE, 0, &vertices.data()[0]);
    glEnableVertexAttribArray(shParamATexCoord);
    glVertexAttribPointer(shParamATexCoord, 2, GL_FLOAT, GL_FALSE, 0, &coords.data()[0]);
    glDrawArrays(GL_TRIANGLES, 0, vertices.size());
}

END_OGLES_GPGPU
