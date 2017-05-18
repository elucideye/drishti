/*!
  @file   GLFWContext.h
  @author David Hirvonen
  @brief  Declaration of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_gltest_GLFWContext_h__
#define __drishti_gltest_GLFWContext_h__

#include "drishti/gltest/drishti_gltest.h"

#include <GLFW/glfw3.h>

DRISHTI_GLTEST_BEGIN

struct GLFWContext
{   
    GLFWContext();
    ~GLFWContext();

    operator bool() const;
    
    GLFWwindow *context = nullptr;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_GLFWContext_h__
