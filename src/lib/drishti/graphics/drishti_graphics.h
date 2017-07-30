/*!
  @file   drishti_graphics.h
  @author David Hirvonen (C++ implementation)
  @brief OpenGL abstractions and GPU macros.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Lineage: ogles_gpgpu/platform/opengl/gl_includes.h
*/

#ifndef __drishti_graphics_drishti_graphics_h__
#define __drishti_graphics_drishti_graphics_h__

// clang-format off
#define DRISHTI_GRAPHICS_BEGIN namespace drishti { namespace graphics {
#define DRISHTI_GRAPHICS_END } }
// clang-format on

// clang-format off
//define something for Windows (64-bit)
#if defined(_WIN32) || defined(_WIN64)
#  include <algorithm> // min/max
#  include <windows.h> // CMakeLists.txt defines NOMINMAX
#  include <gl/glew.h>
#  include <GL/gl.h>
#  define DRISHTI_MSVC 1
#elif __APPLE__
#  include "TargetConditionals.h"
#  if (TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR) || TARGET_OS_IPHONE
#    if defined(DRISHTI_OPENGL_ES3)
#      include <OpenGLES/ES3/gl.h>
#      include <OpenGLES/ES3/glext.h>
#    else
#      include <OpenGLES/ES2/gl.h>
#      include <OpenGLES/ES2/glext.h>
#    endif        
#    define DRISHTI_IOS 1
#  else
#    if defined(DRISHTI_OPENGL_ES3)
#      include <OpenGL/gl3.h>
#      include <OpenGL/gl3ext.h>
#    else
#      include <OpenGL/gl.h>
#      include <OpenGL/glext.h>
#    endif        
#    define DRISHTI_OSX 1
#  endif
#elif defined(__ANDROID__) || defined(ANDROID)
#  if defined(DRISHTI_OPENGL_ES3)
#    include <GLES3/gl3.h>
#    include <GLES3/gl3ext.h>
#  else
#    include <GLES2/gl2.h>
#    include <GLES2/gl2ext.h>
#  endif
#  define DRISHTI_ANDROID 1
#elif defined(__linux__) || defined(__unix__) || defined(__posix__)
#  define GL_GLEXT_PROTOTYPES 1
#  include <GL/gl.h>
#  include <GL/glext.h>
#  define DRISHTI_LINUX 1
#else
#  error platform not supported.
#endif
// clang-format on

#endif
