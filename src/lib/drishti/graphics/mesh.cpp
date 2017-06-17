/*!
  @file   MeshProc
  @author David Hirvonen
  @brief Implementation of ogles_gpgpu shader for drawing lines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/graphics/mesh.h"

BEGIN_OGLES_GPGPU

const char * MeshProc::fshaderPixelSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 void main()
 {
     gl_FragColor = texture2D(uInputTex, vTexCoord);
 });

MeshProc::MeshProc(const Mesh &mesh, const cv::Mat &iso, bool doWire)
{
    if(doWire)
    {
        Mesh::VertexBuffer lines;
        mesh.getWireMeshSegments(lines);
        lineShader = std::make_shared<LineShader>(lines);
    }
    
    if(!iso.empty())
    {
        Mesh::VertexBuffer vertices;
        Mesh::CoordBuffer coords;
        mesh.getTriangleList(vertices, coords);
        meshShader = std::make_shared<MeshShader>(iso, vertices, coords);
    }
}

void MeshProc::setModelViewProjection(const glm::mat4 &mvp)
{
    if(meshShader)
    {
        meshShader->setModelViewProjection(mvp);
    }
    if(lineShader)
    {
        lineShader->setModelViewProjection(mvp);
    }
}

void MeshProc::filterRenderCleanup()
{
    if(meshShader)
    {
        if(background.a)
        {
            glClearColor(background.r, background.g, background.b, background.a);
            glClear(GL_COLOR_BUFFER_BIT);
        }
        meshShader->draw(outFrameW, outFrameH);
    }
    if(lineShader)
    {
        lineShader->draw(outFrameW, outFrameH);
    }
    FilterProcBase::filterRenderCleanup();
}

void Mesh::getTriangleList(VertexBuffer &vb, CoordBuffer &cb) const
{
    vb.reserve(tvi.size() * 3);
    cb.reserve(tvi.size() * 3);
    for (int i = 0; i < tvi.size(); i++)
    {
        const auto &v0 = vertices[tvi[i][0]];
        const auto &v1 = vertices[tvi[i][1]];
        const auto &v2 = vertices[tvi[i][2]];
        vb.push_back(v0);
        vb.push_back(v1);
        vb.push_back(v2);
        
        const auto &p0 = texcoords[tvi[i][0]];
        const auto &p1 = texcoords[tvi[i][1]];
        const auto &p2 = texcoords[tvi[i][2]];
        cb.push_back(p0);
        cb.push_back(p1);
        cb.push_back(p2);
    }
}

void Mesh::getWireMeshSegments(VertexBuffer &vb) const
{
    vb.reserve(tvi.size() * 3);
    for (int i = 0; i < tvi.size(); i++)
    {
        const auto &p0 = vertices[tvi[i][0]];
        const auto &p1 = vertices[tvi[i][1]];
        const auto &p2 = vertices[tvi[i][2]];
        
        vb.push_back(p0);
        vb.push_back(p1);
        
        vb.push_back(p1);
        vb.push_back(p2);
        
        vb.push_back(p2);
        vb.push_back(p0);
    }
}

END_OGLES_GPGPU
