/*!
  @file GLPrinter.cpp
  @author David Hirvonen (C++ implementation)
  @brief Implementation of ogles_gpgpu shader for printing.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/gpu/GLPrinter.h"

#include "drishti/hci/gpu/VeraFont_16_2048.h"
#include "drishti/core/make_unique.h" // add for <= c++11

// clang-format off
#if defined(DRISHTI_OPENGL_ES3)
#  define DRISHTI_GL_RED GL_RED
#else
#  define DRISHTI_GL_RED GL_LUMINANCE
#endif
// clang-format on

BEGIN_OGLES_GPGPU

// clang-format off
const char * GLPrinterShader::vshaderPrinterSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 attribute vec4 position;
 varying vec2 texCoords; 
 void main()
 {
     gl_Position = vec4(position.xy, 0, 1);
     texCoords = position.zw;
 });
// clang-format on

// clang-format off
const char * GLPrinterShader::fshaderPrinterSrc = OG_TO_STR
(
#if defined(OGLES_GPGPU_OPENGLES)
 precision mediump float;
#endif
 varying vec2 texCoords;
 uniform sampler2D tex;
 void main()
 {
     vec4 value = vec4(0.0, 0.0, 0.0, texture2D(tex, texCoords).r);
     gl_FragColor = value; //vec4(0.0, 0.0, 0.0, 1.0);
 });
// clang-format on

GLPrinterShader::GLPrinterShader()
{
    auto& font = Vera_16_2048;

    glGenBuffers(1, &m_vbo);
    Tools::checkGLErr(getProcName(), "glGenBuffers");

    // Load our own texture
    glActiveTexture(GL_TEXTURE0);
    glGenTextures(1, &m_texId);
    glBindTexture(GL_TEXTURE_2D, m_texId);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP_TO_EDGE);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_LINEAR);
    glTexImage2D(GL_TEXTURE_2D, 0, DRISHTI_GL_RED, font.tex_width, font.tex_height, 0, DRISHTI_GL_RED, GL_UNSIGNED_BYTE, font.tex_data);
    glBindTexture(GL_TEXTURE_2D, 0);
    Tools::checkGLErr(getProcName(), "texture init");

    m_shader = drishti::core::make_unique<ogles_gpgpu::Shader>();
    m_shader->buildFromSrc(vshaderPrinterSrc, fshaderPrinterSrc);
    m_shParamAPos = m_shader->getParam(ATTR, "position");
    m_shParamUInputTex = m_shader->getParam(UNIF, "tex");
    Tools::checkGLErr(getProcName(), "shader init");
}

GLPrinterShader::~GLPrinterShader()
{
    glDeleteTextures(1, &m_texId);
    glDeleteBuffers(1, &m_vbo);
}

void GLPrinterShader::begin()
{
    glEnable(GL_BLEND);
    glBlendFunc(GL_SRC_ALPHA, GL_ONE_MINUS_SRC_ALPHA);

    glActiveTexture(GL_TEXTURE0);
    glBindTexture(GL_TEXTURE_2D, m_texId);

    glBindBuffer(GL_ARRAY_BUFFER, m_vbo);

    m_shader->use();
    glUniform1i(m_shParamUInputTex, 0); // set texture unit
}

void GLPrinterShader::printAt(const std::wstring& str, float x, float y, float sx, float sy)
{
    auto& font = Vera_16_2048;

    for (int i = 0; i < str.size(); i++)
    {
        //Find the glyph for the character we are looking for
        texture_glyph_t* glyph = 0;
        for (int j = 0; j < font.glyphs_count; ++j)
        {
            if (font.glyphs[j].codepoint == str[i])
            {
                glyph = &font.glyphs[j];
                break;
            }
        }
        if (!glyph)
        {
            continue;
        }

        // vertex coordinates: x, y
        float x0 = (float)+(x + glyph->offset_x * sx);
        float y0 = (float)+(y + glyph->offset_y * sy);
        float x1 = (float)+(x0 + glyph->width * sx);
        float y1 = (float)+(y0 + glyph->height * sy);

        // texture coordinates: s, t
        float s0 = glyph->s0;
        float t0 = glyph->t0;
        float s1 = glyph->s1;
        float t1 = glyph->t1;

        struct
        {
            float x, y, s, t;
        } data[6] = {
            { x0, y0, s0, t0 },
            { x0, y1, s0, t1 },
            { x1, y1, s1, t1 },
            { x0, y0, s0, t0 },
            { x1, y1, s1, t1 },
            { x1, y0, s1, t0 }
        };

        glEnableVertexAttribArray(m_shParamAPos);
        Tools::checkGLErr(getProcName(), "glEnableVertexAttribArray()");

        glVertexAttribPointer(m_shParamAPos, 4, GL_FLOAT, GL_FALSE, 0, &data[0].x);
        Tools::checkGLErr(getProcName(), "glVertexAttribPointer()");

        glDrawArrays(GL_TRIANGLES, 0, 6);
        Tools::checkGLErr(getProcName(), "glDrawArrays()");

        x += (glyph->advance_x * sx);
        y += (glyph->advance_y * sy);
    }
}

void GLPrinterShader::end()
{
    glBindBuffer(GL_ARRAY_BUFFER, 0);
    glBindTexture(GL_TEXTURE_2D, 0);
    glUseProgram(0);
    Tools::checkGLErr(getProcName(), "end()");
}

END_OGLES_GPGPU
