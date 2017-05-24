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

DRISHTI_GLTEST_BEGIN

class GLContext
{
public:

    enum ContextKind
    {
        kAuto,     // Select most portable context available:
        kGLFW,     // GLFW based (no mobile support)
        kIOS,      // iOS EAGLContext
        kAndroid,  // Android EGL context
        kCount
    };
    
    GLContext() {}
    ~GLContext() {}

    virtual operator bool() const = 0;
    static std::shared_ptr<GLContext> create(ContextKind kind);
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_QGLContext_h__
