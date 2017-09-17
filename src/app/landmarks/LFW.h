/*! -*-c++-*-
  @file   LFW.h
  @author David Hirvonen
  @brief  High level routines for parsing LFW data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_LFW_h__
#define __drishti_landmarks_LFW_h__

#include "landmarks/FACE.h"
#include <fstream>

FACE::Table parseLFW(const std::string& filename);

#endif // __drishti_landmarks_LFW_h__
