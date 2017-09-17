/*! -*-c++-*-
  @file   MUCT.h
  @author David Hirvonen
  @brief  High level routines for parsing MUCT data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_MUCT_h__
#define __drishti_landmarks_MUCT_h__

#include "landmarks/FACE.h"

FACE::Table parseMUCT(const std::string& filename);

#endif // __drishti_landmarks_MUCT_h__
