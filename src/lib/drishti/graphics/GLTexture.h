/*! -*-c++-*-
  @file   GLTexture.h
  @author David Hirvonen
  @brief Declaration of simple OpenGL texture wrapper.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_graphics_GLTexture_h__
#define __drishti_graphics_GLTexture_h__

#include "ogles_gpgpu/common/common_includes.h"

BEGIN_OGLES_GPGPU

struct GLTexture
{
    GLTexture(std::size_t width, std::size_t height, GLenum texType, void* data)
    {
        glGenTextures(1, &texId);
        glBindTexture(GL_TEXTURE_2D, texId);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
        glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
        glPixelStorei(GL_UNPACK_ALIGNMENT, 1);
        glTexImage2D(GL_TEXTURE_2D, 0, GL_RGBA, width, height, 0, texType, GL_UNSIGNED_BYTE, data);
        glBindTexture(GL_TEXTURE_2D, 0);
    }

    ~GLTexture()
    {
        glDeleteTextures(1, &texId);
    }

    operator GLuint() const
    {
        return texId;
    }

    GLuint texId;
};

END_OGLES_GPGPU

#endif // __drishti_graphics_GLTexture_h__
