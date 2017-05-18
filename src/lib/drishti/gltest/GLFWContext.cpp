/*!
  @file   GLFWContext.cpp
  @author David Hirvonen
  @brief  Implementation of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#include "drishti/gltest/GLFWContext.h"

DRISHTI_GLTEST_BEGIN

GLFWContext::GLFWContext()
{
    // initialize glfw context:
    glfwInit();
    glfwWindowHint(GLFW_VISIBLE, GLFW_FALSE);
    context = glfwCreateWindow(640, 480, "", NULL, NULL);
    glfwMakeContextCurrent(context);
    glActiveTexture(GL_TEXTURE0);
}

GLFWContext::~GLFWContext()
{
    glfwDestroyWindow(context);
    glfwTerminate();
}

GLFWContext::operator bool() const
{
    return (context != nullptr);
}

DRISHTI_GLTEST_END
