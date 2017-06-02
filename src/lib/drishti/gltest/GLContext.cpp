#include "drishti/gltest/GLContext.h"

#include <assert.h>

// clang-format off
#if defined(DRISHTI_HAS_GLFW)
#  include "drishti/gltest/GLFWContext.h"
#endif
// clang-format on

// clang-format off
#if defined(DRISHTI_IOS)
#  include "drishti/gltest/GLContextIOS.h"
#endif
// clang-format on

// clang-format off
#if defined(DRISHTI_ANDROID)
#  include "drishti/gltest/GLContextAndroid.h"
#endif
// clang-format on

#include <memory>

DRISHTI_GLTEST_BEGIN

auto GLContext::create(ContextKind kind, const std::string& name, int width, int height) -> GLContextPtr
{
    switch (kind)
    {
        case kAuto:

#if defined(DRISHTI_IOS)
        case kIOS:
            return std::make_shared<drishti::gltest::GLContextIOS>();
#endif

#if defined(DRISHTI_ANDROID)
        case kAndroid:
            return std::make_shared<drishti::gltest::GLContextAndroid>();
#endif

#if defined(DRISHTI_HAS_GLFW)
        case kGLFW:
            return std::make_shared<drishti::gltest::GLFWContext>(name, width, height);
#endif

        default:
            assert(false);
            break;
    }

    return nullptr;
}

DRISHTI_GLTEST_END
