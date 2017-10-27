/*! -*-c++-*-
  @file   meshtex.h
  @author David Hirvonen
  @brief Declaration of texture backed 3d mesh 

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_meshtex_h__
#define __drishti_graphics_meshtex_h__

#include <drishti/graphics/drishti_graphics.h>

#include <glm/glm.hpp>

#include <vector>
#include <array>

DRISHTI_GRAPHICS_BEGIN

struct MeshTex
{
    using VertexBuffer = std::vector<glm::vec4>;
    using CoordBuffer = std::vector<glm::vec2>;

    void getTriangleList(VertexBuffer& vb, CoordBuffer& cb) const;
    void getWireMeshSegments(VertexBuffer& vb) const;

    VertexBuffer vertices;               ///< 3D vertex positions.
    CoordBuffer texcoords;               ///< Texture coordinates for each vertex.
    std::vector<std::array<int, 3>> tvi; ///< Triangle vertex indices
};

DRISHTI_GRAPHICS_END

#endif // __drishti_graphics_meshtex_h__
