/*!
  @file   drishti_graphics.h
  @author David Hirvonen (C++ implementation)
  @brief OpenGL abstractions and GPU macros.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  Lineage: ogles_gpgpu/platform/opengl/gl_includes.h
*/

#ifndef DRISHTI_drishti_graphics_h
#define DRISHTI_drishti_graphics_h

#define DRISHTI_GRAPHICS_BEGIN namespace drishti { namespace graphics {
#define DRISHTI_GRAPHICS_END } }

        //define something for Windows (64-bit)
#if defined(_WIN32) || defined(_WIN64)
#  include <algorithm> // min/max
#  include <windows.h> // CMakeLists.txt defines NOMINMAX
#  include <gl/glew.h>
#  inclyude <GL/gl.h>
#  include <GL/glu.h>
#elif __APPLE__
#  include "TargetConditionals.h"
#  if (TARGET_OS_IPHONE && TARGET_IPHONE_SIMULATOR) || TARGET_OS_IPHONE
#    include <OpenGLES/ES2/gl.h>
#    include <OpenGLES/ES2/glext.h>
#  else
#    include <OpenGL/gl.h>
#    include <OpenGL/glu.h>
#    include <OpenGL/glext.h>
#  endif
#elif defined(__ANDROID__) || defined(ANDROID)
#  include <GLES2/gl2.h>
#  include <GLES2/gl2ext.h>
#elif defined(__linux__) || defined(__unix__) || defined(__posix__)
#  define GL_GLEXT_PROTOTYPES 1
#  include <GL/gl.h>
#  include <GL/glu.h>
#  include <GL/glext.h>
#else
#  error platform not supported.
#endif

#endif
