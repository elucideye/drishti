/*! -*-c++-*-
  @file   BIOID.h
  @author David Hirvonen
  @brief  High level routines for parsing BIOID data

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_BIOID_h__
#define __drishti_landmarks_BIOID_h__

#include "landmarks/FACE.h"

#include <fstream>

FACE::Table parseBIOID(const std::string& filename);

#endif // __drishti_landmarks_BIOID_h__
