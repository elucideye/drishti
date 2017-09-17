/*! -*-c++-*-
  @file   TWO.h
  @author David Hirvonen
  @brief  High level routines for parsing two line landmark data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_TWO_h__
#define __drishti_landmarks_TWO_h__

#include "landmarks/FACE.h"

#include <fstream>

FACE::Table parseTWO(const std::string& filename);

#endif // __drishti_landmarks_TWO_h__
