/*!
  @file   drishti_graphics.h
  @author David Hirvonen (C++ implementation) <dhirvonen elucideye com>
  @brief OpenGL abstractions and GPU macros.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_drishti_graphics_h
#define DRISHTI_drishti_graphics_h

#define DRISHTI_GRAPHICS_BEGIN namespace drishti { namespace graphics {
#define DRISHTI_GRAPHICS_END } }

#ifdef _WIN64
//define something for Windows (64-bit)
#elif _WIN32
//define something for Windows (32-bit)
#elif __APPLE__
#include "TargetConditionals.h"
#if TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR
#  define DRISHTI_IOS 1
// define something for simulator
#elif TARGET_OS_IPHONE
#  define DRISHTI_IOS 1
#  include <OpenGLES/ES2/gl.h>
#  include <OpenGLES/ES2/glext.h>
#  include <arm_neon.h>
#  define DO_GRAPHICS_SIMD 1
#else
#  include <OpenGL/gl.h>
#  include <OpenGL/glext.h>
#  define DO_GRAPHICS_SIMD 0
#endif
#elif __ANDROID__ || ANDROID
//#include <GLES3/gl3.h>
//#include <GLES3/gl3ext.h>
#include <GLES2/gl2.h>
#include <GLES2/gl2ext.h>
#elif __linux
// linux
#elif __unix // all unices not caught above
// Unix
#elif __posix
// POSIX
#endif

#if !defined(DRISHTI_IOS)
# define DRISHTI_IOS 0
#endif

// ANDROID
//#include <GLES2/gl2.h>
//#include <GLES2/gl2ext.h>

//#include <GLES/gl.h>
//#include <GLES/glext.h>

#include <memory>

// http://clean-cpp.org/underprivileged-unique-pointers-make_unique/#more-54
template <typename Value, typename ... Arguments>
std::unique_ptr<Value> make_unique(Arguments && ... arguments_for_constructor)
{
    return std::unique_ptr<Value>(new Value(std::forward<Arguments>(arguments_for_constructor)...));
}

#endif
