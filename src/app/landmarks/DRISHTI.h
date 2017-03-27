/*!
  @file   DRISHTI.h
  @author David Hirvonen
  @brief  High level routines for parsing DRISHTI data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __landmarks_DRISHTI_h__
#define __landmarks_DRISHTI_h__

#include "landmarks/FACE.h"
#include <fstream>

void parseDRISHTI(const std::string &filename, FACE::record &output);
FACE::Table parseDRISHTI(const std::string &filename);

#endif // __landmarks_DRISHTI_h__

