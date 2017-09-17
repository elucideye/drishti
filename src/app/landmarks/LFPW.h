/*! -*-c++-*-
  @file   LFPW.h
  @author David Hirvonen
  @brief  High level routines for parsing LFPW data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_landmarks_LFPW_h__
#define __drishti_landmarks_LFPW_h__

#include "landmarks/FACE.h"
#include <fstream>

// find lfpw/trainset -name "*.png" -or -name "*.pts" | awk '/.pts/ {print >"/tmp/pts.txt"}; /.png/ {print >"/tmp/png.txt"}'
// sed -E 's|.*/(.*).pts|\1 &|g' /tmp/pts.txt | sort > /tmp/pts1.txt
// sed -E 's|.*/(.*).png|\1 &|g' /tmp/png.txt | sort > /tmp/png1.txt
// join /tmp/png1.txt /tmp/pts1.txt | cut -f2- -d' ' > png_pts.txt

// Note: This format is similar to the BIOID data so the same
// back end grammar is used.

// Input: two column textu file in format
//
// image_filename1.png landmark_filename1.pts
// image_filename2.png landmark_filename2.pts
// image_filename3.png landmark_filename3.pts
// ...
// image_filenamen.png landmark_filenamen.pts

FACE::Table parseLFPW(const std::string& filename);

#endif // __drishti_landmarks_LFPW_h__
