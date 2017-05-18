#include "drishti/gltest/GLContext.h"

#include <assert.h>

#if defined(DRISHTI_HAS_GLFW)
#  include "drishti/gltest/GLFWContext.h"
#endif

#if defined(DRISHTI_BUILD_QT)
#  include "drishti/gltest/QGLContext.h"
#endif

#include <memory>

DRISHTI_GLTEST_BEGIN

std::shared_ptr<GLContext> GLContext::create(ContextKind kind)
{
    switch (kind)
    {
       case kAuto:
#if defined(DRISHTI_BUILD_QT)
       case kQT:
           return std::make_shared<drishti::gltest::QGLContext>();
           break;
#endif
#if defined(DRISHTI_HAS_GLFW)           
       case kGLFW:
           return std::make_shared<drishti::gltest::GLFWContext>();
           break;
#endif
       default:
           assert(false);
           break;
    }

    return nullptr;
}

DRISHTI_GLTEST_END
