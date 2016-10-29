/**
  @file   drishti_sdk.hpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains macros used to control public symbols visibility -- the symbols
  names themselves will be visible if RTTI and exceptions are enabled.
*/

#ifndef __drishtisdk__SDK__
#define __drishtisdk__SDK__

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)
# ifdef drishti_EXPORTS
#  define DRISHTI_EXPORTS __declspec(dllexport)
# else
#  define DRISHTI_EXPORTS __declspec(dllimport)
# endif
#elif (defined __GNUC__ && __GNUC__ >= 4) || defined(__clang__)
#  define DRISHTI_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define DRISHTI_EXPORTS
#endif

#define _DRISHTI_BEGIN namespace drishti {
#define _DRISHTI_END }

#define _DRISHTI_SDK_BEGIN _DRISHTI_BEGIN namespace sdk {
#define _DRISHTI_SDK_END _DRISHTI_END }

#include <string> // move out

_DRISHTI_SDK_BEGIN

#define DRISHTI_VERSION_MAJOR 0
#define DRISHTI_VERSION_MINOR 7
#define DRISHTI_VERSION_PATCH 2
#define DRISHTI_VERSION_STATUS "" // e.g., "-dev"

#define DRISHTI_STR_EXP(__A)  #__A
#define DRISHTI_STR(__A)      DRISHTI_STR_EXP(__A)

#define DRISHTI_STRW_EXP(__A)  L#__A
#define DRISHTI_STRW(__A)      DRISHTI_STRW_EXP(__A)

#define DRISHTI_VERSION  DRISHTI_STR(DRISHTI_VERSION_MAJOR) "." DRISHTI_STR(DRISHTI_VERSION_MINOR) "." DRISHTI_STR(DRISHTI_VERSION_REVISION) DRISHTI_VERSION_STATUS

#define DRISHTI_EXTERN_C_BEGIN extern "C" {
#define DRISHTI_EXTERN_C_END }

std::string getVersion();

_DRISHTI_SDK_END

#endif // __drishtisdk__SDK__
