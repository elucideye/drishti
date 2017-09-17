/*! -*-c++-*-
  @file   drawing.h
  @author David Hirvonen
  @brief  Declaration of simple drawing routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drawing_h__
#define __drishti_core_drawing_h__

#include <iostream>
#include "drishti_core.h"
#include <opencv2/core/core.hpp>

DRISHTI_CORE_NAMESPACE_BEGIN

void quiver(cv::Mat3b& canvas, const cv::Mat1f& dx, const cv::Mat1f& dy, int step, float scale, const cv::Mat1b& mask = cv::Mat1b());

DRISHTI_CORE_NAMESPACE_END

#endif /* defined(__drishti_core_drawing_h__) */
