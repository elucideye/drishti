/*! -*-c++-*- 
  @file GLCircle.h @author David Hirvonen (C++ implementation)
  @brief Declaration of ogles_gpgpu shader for rendering circles.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef OGLES_GPGPU_COMMON_GL_CIRCLE_PROC
#define OGLES_GPGPU_COMMON_GL_CIRCLE_PROC

#include "ogles_gpgpu/common/proc/base/filterprocbase.h"

#include <string>
#include <memory>

BEGIN_OGLES_GPGPU

// TODO: add to ogles_gpgpu
struct Vec2f
{
    Vec2f() {}
    Vec2f(float a, float b)
    {
        data[0] = a;
        data[1] = b;
    }
    GLfloat data[2];
};

class CircleProc : public ogles_gpgpu::FilterProcBase
{
public:
    CircleProc() {}
    virtual const char* getProcName()
    {
        return "CircleProc";
    }

    void setColor(const Vec3f& value) { color = value; }
    void setRadius(float value) { radius = value; }
    void setCenter(const Vec2f& value) { center = value; }

private:
    virtual const char* getFragmentShaderSource()
    {
        return fshaderCircleSrc;
    }
    virtual void getUniforms()
    {
        shParamUCenter = shader->getParam(UNIF, "center");
        shParamURadius = shader->getParam(UNIF, "radius");
        shParamUColor = shader->getParam(UNIF, "color");
        shParamURatio = shader->getParam(UNIF, "ratio");
    }
    virtual void setUniforms()
    {
        ratio = static_cast<float>(getOutFrameW()) / static_cast<float>(getOutFrameH());
        glUniform2fv(shParamUCenter, 1, &center.data[0]);
        glUniform1f(shParamURadius, radius);
        glUniform3fv(shParamUColor, 1, &color.data[0]);
        glUniform1f(shParamURatio, ratio);
    }
    static const char* fshaderCircleSrc; // fragment shader source

    float ratio = 1.f;
    Vec3f color = { 0.f, 1.f, 0.f };
    Vec2f center = { 0.5f, 0.5f };
    float radius = 0.1f;

    GLint shParamUCenter;
    GLint shParamURadius;
    GLint shParamUColor;
    GLint shParamURatio;
};

END_OGLES_GPGPU

#endif
