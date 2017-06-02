/*!
  @file   GLContext.h
  @author David Hirvonen
  @brief  Declaration of a cross platform offscreen OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_gltest_GLContext_h__
#define __drishti_gltest_GLContext_h__

#include "drishti/gltest/drishti_gltest.h"
#include <memory>
#include <string>
#include <functional>

DRISHTI_GLTEST_BEGIN

class GLContext
{
public:
    using GLContextPtr = std::shared_ptr<GLContext>;
    using RenderDelegate = std::function<bool(void)>;

    enum ContextKind
    {
        kAuto,    // Select most portable context available:
        kGLFW,    // GLFW based (no mobile support)
        kIOS,     // iOS EAGLContext
        kAndroid, // Android EGL context
        kCount
    };

    struct Geometry
    {
        int width = 0;
        int height = 0;
        float tx = 0.f;
        float ty = 0.f;
        float sx = 1.f;
        float sy = 1.f;
    };

    GLContext() {}
    ~GLContext() {}

    GLContext(const std::string& name, int width, int height) {}

    virtual operator bool() const = 0;
    virtual void operator()() {} // make current

    virtual bool hasDisplay() const = 0;
    virtual void resize(int width, int height) {}
    virtual void operator()(RenderDelegate& f){}; // render loop

    Geometry& getGeometry() { return m_geometry; }
    const Geometry& getGeometry() const { return m_geometry; }

    Geometry m_geometry;

    // Create context (w/ window if name is specified):
    static GLContextPtr create(ContextKind kind, const std::string& name = {}, int width = 640, int height = 480);
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_QGLContext_h__
