/*! -*-c++-*-
  @file   HELEN.h
  @author David Hirvonen
  @brief  High level routines for (de)serialization and manipulation of flattened HELEN data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_HELEN_h__
#define __drishti_landmarks_HELEN_h__

#include "landmarks/FACE.h"
#include <fstream>

FACE::Table parseHELEN(const std::string& filename);

#endif // __drishti_landmarks_HELEN_h__
