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
    for (auto i : tvi)
    {
        const auto& v0 = vertices[i[0]];
        const auto& v1 = vertices[i[1]];
        const auto& v2 = vertices[i[2]];
        vb.push_back(v0);
        vb.push_back(v1);
        vb.push_back(v2);

        const auto& p0 = texcoords[i[0]];
        const auto& p1 = texcoords[i[1]];
        const auto& p2 = texcoords[i[2]];
        cb.push_back(p0);
        cb.push_back(p1);
        cb.push_back(p2);
    }
}

void MeshTex::getWireMeshSegments(VertexBuffer& vb) const
{
    vb.reserve(tvi.size() * 3);
    for (auto i : tvi)
    {
        const auto& p0 = vertices[i[0]];
        const auto& p1 = vertices[i[1]];
        const auto& p2 = vertices[i[2]];

        vb.push_back(p0);
        vb.push_back(p1);

        vb.push_back(p1);
        vb.push_back(p2);

        vb.push_back(p2);
        vb.push_back(p0);
    }
}

DRISHTI_GRAPHICS_END
