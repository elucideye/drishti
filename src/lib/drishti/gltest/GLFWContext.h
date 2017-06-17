/*!
  @file   GLFWContext.h
  @author David Hirvonen
  @brief  Declaration of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_gltest_GLFWContext_h__
#define __drishti_gltest_GLFWContext_h__

#include "drishti/gltest/GLContext.h"

// clang-format off
#if defined(_WIN32) || defined(_WIN64)
#  include <windows.h> // CMakeLists.txt defines NOMINMAX
#  include <gl/glew.h>
#endif
// clang-format on

#include <GLFW/glfw3.h>

DRISHTI_GLTEST_BEGIN

struct GLFWContextPool;

class GLFWContext : public GLContext
{
public:
    GLFWContext(const std::string& name = {}, int width = 640, int height = 480);
    ~GLFWContext();

    virtual void operator()();
    virtual operator bool() const;

    // Display related:
    virtual void setCursorCallback(const CursorDelegate &callback);
    virtual void setCursorVisibility(bool flag);
    virtual void setCursor(double x, double y);
    virtual void getCursor(double &x, double &y);
    virtual void setWait(bool wait) { m_wait = wait; }
    virtual bool hasDisplay() const;
    virtual void resize(int width, int height);
    virtual void operator()(std::function<bool(void)>& f);

    GLFWwindow* getContext() const { return m_context; }
    void framebufferSizeCallback(int width, int height);

protected:
    friend GLFWContextPool;
    void alloc(const std::string& name = {}, int width = 640, int height = 480);

    GLFWwindow* m_context = nullptr;
    bool m_visible = false;
    bool m_showCursor = false;
    bool m_wait = false;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_GLFWContext_h__
