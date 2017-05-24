/*!
  @file   GLContextIOS
  @author David Hirvonen
  @brief  Declaration of minimal "hidden" OpenGL context for IOS.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_gltest_GLContextIOS_h__
#define __drishti_gltest_GLContextIOS_h__

#include "drishti/gltest/GLContext.h"

#include <memory>

DRISHTI_GLTEST_BEGIN

class GLContextIOS : public GLContext
{
public:
    
    GLContextIOS();
    ~GLContextIOS();
    operator bool() const;

private:
    
    struct Impl;
    std::unique_ptr<Impl> impl;
};

DRISHTI_GLTEST_END

#endif // __drishti_gltest_GLContextIOS_h__
