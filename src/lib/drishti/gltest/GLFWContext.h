/*!
  @file   GLFWContext.h
  @author David Hirvonen
  @brief  Declaration of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_gltest_GLFWContext_h__
#define __drishti_gltest_GLFWContext_h__

#include "drishti/gltest/GLContext.h"

#if defined(_WIN32) || defined(_WIN64)
#  include <windows.h> // CMakeLists.txt defines NOMINMAX
#  include <gl/glew.h>
#endif

#include <GLFW/glfw3.h>

DRISHTI_GLTEST_BEGIN

struct GLFWContext : public GLContext
{   
    GLFWContext();
    ~GLFWContext();
    virtual void operator()();
    operator bool() const;
    
    GLFWwindow *context = nullptr;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_GLFWContext_h__
