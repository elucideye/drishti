/*! -*-c++-*-
  @file GLPrinter.h
  @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for printing.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_COMMON_GL_PRINTER_PROC
#define OGLES_GPGPU_COMMON_GL_PRINTER_PROC

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

#include <string>
#include <memory>

BEGIN_OGLES_GPGPU

class GLPrinterShader
{
public:
    GLPrinterShader();
    ~GLPrinterShader();

    static const char* getProcName() { return "GLPrinterShader"; }

    void begin();
    void end();
    void printAt(const std::wstring& str, float x, float y, float sx, float sy);

    GLuint m_texId;
    GLuint m_vbo, m_vao;

    // #### Draw shader ####
    std::shared_ptr<Shader> m_shader;
    GLuint m_shParamAPos;
    GLuint m_shParamATexCoord;
    GLuint m_shParamUInputTex;

    static const char* vshaderPrinterSrc; // vertex shader source
    static const char* fshaderPrinterSrc; // fragment shader source
};

END_OGLES_GPGPU

#endif
