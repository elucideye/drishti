/*! -*-c++-*-
  @file   meshtex.cpp
  @author David Hirvonen
  @brief Declaration of texture backed 3d mesh 

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <drishti/graphics/meshtex.h>

DRISHTI_GRAPHICS_BEGIN

void MeshTex::getTriangleList(VertexBuffer& vb, CoordBuffer& cb) const
{
    vb.reserve(tvi.size() * 3);
    cb.reserve(tvi.size() * 3);
    for (int i = 0; i < tvi.size(); i++)
    {
        const auto& v0 = vertices[tvi[i][0]];
        const auto& v1 = vertices[tvi[i][1]];
        const auto& v2 = vertices[tvi[i][2]];
        vb.push_back(v0);
        vb.push_back(v1);
        vb.push_back(v2);

        const auto& p0 = texcoords[tvi[i][0]];
        const auto& p1 = texcoords[tvi[i][1]];
        const auto& p2 = texcoords[tvi[i][2]];
        cb.push_back(p0);
        cb.push_back(p1);
        cb.push_back(p2);
    }
}

void MeshTex::getWireMeshSegments(VertexBuffer& vb) const
{
    vb.reserve(tvi.size() * 3);
    for (int i = 0; i < tvi.size(); i++)
    {
        const auto& p0 = vertices[tvi[i][0]];
        const auto& p1 = vertices[tvi[i][1]];
        const auto& p2 = vertices[tvi[i][2]];

        vb.push_back(p0);
        vb.push_back(p1);

        vb.push_back(p1);
        vb.push_back(p2);

        vb.push_back(p2);
        vb.push_back(p0);
    }
}

DRISHTI_GRAPHICS_END
