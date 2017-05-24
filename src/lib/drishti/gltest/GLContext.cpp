#include "drishti/gltest/GLContext.h"

#include <assert.h>

#if defined(DRISHTI_HAS_GLFW)
#  include "drishti/gltest/GLFWContext.h"
#endif

#if defined(DRISHTI_IOS)
#  include "drishti/gltest/GLContextIOS.h"
#endif

#if defined(DRISHTI_ANDROID)
#  include "drishti/gltest/GLContextAndroid.h"
#endif

#include <memory>

DRISHTI_GLTEST_BEGIN

std::shared_ptr<GLContext> GLContext::create(ContextKind kind)
{
    switch (kind)
    {
       case kAuto:

#if defined(DRISHTI_IOS)
       case kIOS: return std::make_shared<drishti::gltest::GLContextIOS>();
#endif           

#if defined(DRISHTI_ANDROID)
       case kAndroid: return std::make_shared<drishti::gltest::GLContextAndroid>();
#endif

#if defined(DRISHTI_HAS_GLFW)
        case kGLFW: return std::make_shared<drishti::gltest::GLFWContext>();
#endif

       default:
           assert(false);
           break;
    }

    return nullptr;
}

DRISHTI_GLTEST_END
