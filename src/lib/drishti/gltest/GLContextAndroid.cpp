/*!
  @file   GLContextAndroid.cpp
  @author David Hirvonen
  @brief  Implementation of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#include "drishti/gltest/GLContextAndroid.h"
#include <iostream>

DRISHTI_GLTEST_BEGIN

GLContextAndroid::GLContextAndroid(int width, int height)
{
    // EGL config attributes
    const EGLint confAttr[] =
    {
        EGL_RENDERABLE_TYPE, EGL_OPENGL_ES2_BIT,    // very important!
        EGL_SURFACE_TYPE, EGL_PBUFFER_BIT,          // we will create a pixelbuffer surface
        EGL_RED_SIZE,   8,
        EGL_GREEN_SIZE, 8,
        EGL_BLUE_SIZE,  8,
        EGL_ALPHA_SIZE, 8,     // if you need the alpha channel
        EGL_DEPTH_SIZE, 16,    // if you need the depth buffer
        EGL_NONE
    };

    // EGL context attributes
    const EGLint ctxAttr[] =
    {
        EGL_CONTEXT_CLIENT_VERSION, 2,              // very important!
        EGL_NONE
    };

    // surface attributes
    // the surface size is set to the input frame size
    const EGLint surfaceAttr[] =
    {
        EGL_WIDTH, width,
        EGL_HEIGHT, height,
        EGL_NONE
    };
    
    EGLint eglMajVers, eglMinVers;
    EGLint numConfigs;

    eglDisp = eglGetDisplay(EGL_DEFAULT_DISPLAY);
    eglInitialize(eglDisp, &eglMajVers, &eglMinVers);
    eglChooseConfig(eglDisp, confAttr, &eglConf, 1, &numConfigs);
    eglCtx = eglCreateContext(eglDisp, eglConf, EGL_NO_CONTEXT, ctxAttr);
    eglSurface = eglCreatePbufferSurface(eglDisp, eglConf, surfaceAttr);    
    eglMakeCurrent(eglDisp, eglSurface, eglSurface, eglCtx);
}

GLContextAndroid::~GLContextAndroid()
{
    eglMakeCurrent(eglDisp, EGL_NO_SURFACE, EGL_NO_SURFACE, EGL_NO_CONTEXT);
    eglDestroyContext(eglDisp, eglCtx);
    eglDestroySurface(eglDisp, eglSurface);
    eglTerminate(eglDisp);

    eglDisp = EGL_NO_DISPLAY;
    eglSurface = EGL_NO_SURFACE;
    eglCtx = EGL_NO_CONTEXT;
}

GLContextAndroid::operator bool() const
{
    return (eglCtx != EGL_NO_CONTEXT);
}

DRISHTI_GLTEST_END
