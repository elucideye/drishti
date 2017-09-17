/*! -*-c++-*-
  @file   drishti_defs.hpp
  @author David Hirvonen
  @brief  Drishti cross platform export header.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_defs_hpp__
#define __drishti_core_drishti_defs_hpp__

// clang-format off
#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__)
# ifdef drishti_EXPORTS
#  define DSDK_EXPORTS __declspec(dllexport)
# else
#  define DSDK_EXPORTS __declspec(dllimport)
# endif
#elif (defined __GNUC__ && __GNUC__ >= 4) || defined(__clang__)
#  define DSDK_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define DSDK_EXPORTS
#endif
// clang-format on

#endif
