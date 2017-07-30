/**
  @file   drishti_gl.hpp
  @author David Hirvonen
  @brief  Provide standard OpenGL includes on various platforms.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file provides standard OpenGL includes. 

  Lineage: ogles_gpgpu/platform/opengl/gl_includes.h
*/

#ifndef __drishti_drishti_drishti_gl_hpp__
#define __drishti_drishti_drishti_gl_hpp__ 1

// clang-format off
//define something for Windows (64-bit)
#if defined(_WIN32) || defined(_WIN64)
#  include <algorithm> // min/max
#  include <windows.h> // CMakeLists.txt defines NOMINMAX
#  include <gl/glew.h>
#  include <GL/gl.h>
#  include <GL/glu.h>
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
#  else
#    if defined(DRISHTI_OPENGL_ES3)
#      include <OpenGL/gl3.h>
#      include <OpenGL/gl3ext.h>
#    else
#      include <OpenGL/gl.h>
#      include <OpenGL/glext.h>
#    endif        
#  endif
#elif defined(__ANDROID__) || defined(ANDROID)
#  if defined(DRISHTI_OPENGL_ES3)
#    include <GLES3/gl3.h>
#    include <GLES3/gl3ext.h>
#  else
#    include <GLES2/gl2.h>
#    include <GLES2/gl2ext.h>
#  endif
#elif defined(__linux__) || defined(__unix__) || defined(__posix__)
#  define GL_GLEXT_PROTOTYPES 1
#  include <GL/gl.h>
#  include <GL/glext.h>
#else
#  error platform not supported.
#endif
// clang-format on

#endif // __drishti_drishti_drishti_gl_hpp__
