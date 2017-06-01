/*!
  @file   GLWindow.h
  @author David Hirvonen
  @brief  OpenGL preview window declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_GLWindow_h__
#define __drishti_hci_GLWindow_h__

#include <GLFW/glfw3.h>

#include <string>
#include <functional>

class GLWindow
{
public:
    
    struct Window
    {
        int width;
        int height;
        float tx;
        float ty;
        float sx;
        float sy;
    };
    
    static Window impl;

    GLWindow(const std::string &name, int width, int height);
    void resize(int width, int height);
    void operator()();
    void operator()(std::function<bool(void)> &f);
    
private:
    
    GLFWwindow *window = nullptr;
};

#endif // __drishti_hci_GLWindow_h__
