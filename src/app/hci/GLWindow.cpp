/*!
  @file   GLWindow.cpp
  @author David Hirvonen
  @brief  OpenGL preview window implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "GLWindow.h"

GLWindow::Window GLWindow::impl; // declaration

static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    GLWindow::impl.sx = static_cast<float>(width) / GLWindow::impl.width;
    GLWindow::impl.sy = static_cast<float>(height) / GLWindow::impl.height;
}

GLWindow::GLWindow(const std::string &name, int width, int height)
{
    impl = { width, height, 2.f, 2.f };
        
    if(!glfwInit())
    {
        glfwTerminate();
        throw std::runtime_error("glfwInit()");
    }

    window = glfwCreateWindow(width, height, name.c_str(), nullptr, nullptr);
    if(!window)
    {
        glfwTerminate();
        throw std::runtime_error("glfwCreateWindow()");
    }
    
    glfwSetFramebufferSizeCallback(window, framebuffer_size_callback);
    glfwMakeContextCurrent(window);
    glfwGetFramebufferSize(window, &width, &height);
    framebuffer_size_callback(window, width, height);
}

void GLWindow::operator()(std::function<bool(void)> &f)
{
    bool okay = true;
    while(!glfwWindowShouldClose(window) && okay)
    {
        glfwPollEvents();
        okay = f(); // <== callback
        glfwSwapBuffers(window);
    }
    glfwTerminate();
}


