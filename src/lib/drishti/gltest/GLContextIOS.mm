/*!
  @file   GLContextIOS.cpp
  @author David Hirvonen
  @brief  Implementation of minimal "hidden" GLFW based OpenGL context.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#include "drishti/gltest/GLContextIOS.h"
#include "drishti/core/make_unique.h"

#import <UIKit/UIKit.h>

//#include <OpenGLES/EAGL.h>
//#include <OpenGLES/ES2/gl.h>
//#include <OpenGLES/ES2/glext.h>

DRISHTI_GLTEST_BEGIN

struct GLContextIOS::Impl
{
    Impl()
    {
        egl = [[EAGLContext alloc] initWithAPI:kEAGLRenderingAPIOpenGLES2];
        [EAGLContext setCurrentContext:egl];
    }
    
    EAGLContext *egl = nullptr;
};

GLContextIOS::GLContextIOS()
{
    impl = drishti::core::make_unique<Impl>();
}

GLContextIOS::~GLContextIOS()
{
    
}

GLContextIOS::operator bool() const
{
    return (impl && impl->egl);
}

DRISHTI_GLTEST_END

