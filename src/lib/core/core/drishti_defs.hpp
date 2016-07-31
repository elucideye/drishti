/*!
  @file   drishti_defs.hpp
  @author David Hirvonen
  @brief  Drishti cross platform export header.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_DEFS_H
#define DRISHTI_DEFS_H

#if (defined WIN32 || defined _WIN32 || defined WINCE || defined __CYGWIN__) && defined CVAPI_EXPORTS
#  define DSDK_EXPORTS __declspec(dllexport)
#elif (defined __GNUC__ && __GNUC__ >= 4) || defined(__clang__)
#  define DSDK_EXPORTS __attribute__ ((visibility ("default")))
#else
#  define DSDK_EXPORTS
#endif

#endif

