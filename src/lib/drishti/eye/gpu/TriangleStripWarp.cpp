/*! -*-c++-*-
  @file   TriangleStripWarp.cpp
  @author David Hirvonen
  @brief  Implementation of utility structure to perform arbitrary warps via GL_TRIANGL_STRIPS

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/gpu/TriangleStripWarp.h"

BEGIN_OGLES_GPGPU

// clang-format off
const char * TriangleStripWarp::fshaderTriangleSrc = 
#if defined(OGLES_GPGPU_OPENGLES)
OG_TO_STR(precision mediump float;)
#endif
OG_TO_STR(
 varying vec2 vTexCoord;
 uniform sampler2D uInputTex;
 uniform float gain;
 void main()
 {
     vec4 val = texture2D(uInputTex, vTexCoord);
     gl_FragColor = val;
});
// clang-format on

// clang-format off
//const char *FilterProcBase::vshaderDefault = OG_TO_STR(
//attribute vec4 aPos;
//attribute vec2 aTexCoord;
//varying vec2 vTexCoord;
//void main()
//{
//    gl_Position = aPos;
//    vTexCoord = aTexCoord;
//});
// clang-format on

TriangleStripWarp::TriangleStripWarp() {}

// We will override texture and vertex coords:
void TriangleStripWarp::filterRenderSetCoords()
{
    // render to FBO
    if (fbo)
    {
        fbo->bind();
    }

    // set geometry
    glEnableVertexAttribArray(shParamAPos);

    // Destination point in clip space:
    glVertexAttribPointer(shParamAPos, 2, GL_FLOAT, GL_FALSE, 0, &m_texels[0]); // [(-1,-1)  (+1,+1)]

    // Source point in texture space:
    glVertexAttribPointer(shParamATexCoord, 2, GL_FLOAT, GL_FALSE, 0, &m_pixels[0]); // [(0,0)  (1,1)]
    glEnableVertexAttribArray(shParamATexCoord);
}

void TriangleStripWarp::filterRenderDraw()
{
    glDrawArrays(GL_TRIANGLE_STRIP, 0, static_cast<GLsizei>(m_pixels.size()));
}

// set geometry
//glEnableVertexAttribArray(shParamAPos);
//glVertexAttribPointer(shParamAPos, OGLES_GPGPU_QUAD_COORDS_PER_VERTEX, GL_FLOAT, GL_FALSE, 0, &ProcBase::quadVertices[0]);

//glVertexAttribPointer(shParamATexCoord, OGLES_GPGPU_QUAD_TEXCOORDS_PER_VERTEX, GL_FLOAT, GL_FALSE, 0, &ProcBase::quadTexCoordsStd[0]);
//glEnableVertexAttribArray(shParamATexCoord);

END_OGLES_GPGPU
