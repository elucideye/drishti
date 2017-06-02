/*!
  @file   GLWindow.cpp
  @author David Hirvonen
  @brief  OpenGL preview window implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "GLWindow.h"
#include <cmath>

GLWindow::Window GLWindow::impl; // declaration

static void framebuffer_size_callback(GLFWwindow* window, int width, int height)
{
    const float wScale = static_cast<float>(width) / static_cast<float>(GLWindow::impl.width);
    const float hScale = static_cast<float>(height) / static_cast<float>(GLWindow::impl.height);
    const float minScale = (wScale < hScale) ? wScale : hScale;
    const float winWidth = minScale * GLWindow::impl.width;
    const float winHeight = minScale * GLWindow::impl.height;
    const int wShift = static_cast<int>(std::nearbyint((width - winWidth) / 2.0f));
    const int hShift = static_cast<int>(std::nearbyint((height - winHeight) / 2.0f));

    GLWindow::impl.sx = minScale;
    GLWindow::impl.sy = minScale;
    GLWindow::impl.tx = wShift;
    GLWindow::impl.ty = hShift;
    
    glfwMakeContextCurrent(window);
    glViewport(wShift, hShift, std::nearbyint(winWidth), std::nearbyint(winHeight));
    //render_callback(window);
}

GLWindow::GLWindow(const std::string &name, int width, int height)
{
    impl = { width, height, 0.f, 0.f, 2.f, 2.f };
        
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
    glfwShowWindow(window);
}

void GLWindow::operator()()
{
    glfwMakeContextCurrent(window);
}

void GLWindow::resize(int width, int height)
{
    impl.width = width;
    impl.height = height;
    glfwSetWindowSize(window, width, height);
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


