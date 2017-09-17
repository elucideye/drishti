/*! -*-c++-*-
  @file   FACE.cpp
  @author David Hirvonen
  @brief  High level routines for parsing face data.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Comment/uncomment line below to toggle verbose output in boost:spirit (BNF parser)
//#define BOOST_SPIRIT_DEBUG

#include "landmarks/FACE.h"
#include "landmarks/DRISHTI.h"

#include "drishti/core/Line.h"
#include "drishti/core/infix_iterator.h"
#include "drishti/core/string_utils.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/face/face_util.h"

#include <opencv2/highgui.hpp>

// cereal:

#define FACE_DRAW_WITH_EYES 0
#define FACE_SHOW_POINTS 0

//#include "dlib/data_io/image_dataset_metadata.h"

//// Write file header:
// file << drishti::core::getXMLHeader();

//count++;
//std::string name = drishti::core::basename(line.filename);
//std::stringstream ss;
//ss << output << "/" << name << "_" << count;
//
//// Write the synteiszed face crop:
//cv::imwrite(ss.str() + ".png", crop);
//
//// Write the xml entry:
// drishti::core::AnnotatedContourImage metadata;
// metadata.filename = ss.str() + ".png";
// metadata.contours.push_back(shape);
// file << metadata.getLandmarksAsString("\t");
// file << drishti::core::getXMLFooter();
