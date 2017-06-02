/*!
  @file   GLContextAndroid
  @author David Hirvonen
  @brief  Declaration of minimal "hidden" OpenGL context for Android.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_gltest_GLContextAndroid_h__
#define __drishti_gltest_GLContextAndroid_h__

#include "drishti/gltest/GLContext.h"

#include <EGL/egl.h>
#include <android/log.h>
#include <android/window.h>
#include <dlfcn.h>
#include <errno.h>
#include <math.h>
#include <stdlib.h>
#include <stdio.h>

DRISHTI_GLTEST_BEGIN

struct GLContextAndroid : public GLContext
{
    GLContextAndroid(int width = 640, int height = 480);
    ~GLContextAndroid();

    virtual operator bool() const;
    virtual void operator()() {} // make current

    virtual bool hasDisplay() const;
    virtual void resize(int width, int height);
    virtual void operator()(std::function<bool(void)>& f);    

    EGLConfig eglConf;
    EGLSurface eglSurface = EGL_NO_SURFACE;
    EGLContext eglCtx = EGL_NO_CONTEXT;
    EGLDisplay eglDisp = EGL_NO_DISPLAY;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_GLContextAndroid_h__
