/*! -*-c++-*-
  @file   padding.h
  @author David Hirvonen
  @brief  Declaration of simple aspect ratio aware padding routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"
#include <opencv2/core/core.hpp>

#ifndef __drishti_core_padding_h__
#define __drishti_core_padding_h__

DRISHTI_CORE_NAMESPACE_BEGIN

cv::Point padWithInpainting(const cv::Mat& image, cv::Mat& padded, int top, int bottom, int left, int right, bool inPaint = true);
cv::Point padToAspectRatio(const cv::Mat& image, cv::Mat& padded, double aspectRatio, bool inPaint = true);
cv::Point padToWidthUsingAspectRatio(const cv::Mat& canvas, cv::Mat& padded, int width, double aspectRatio, bool inPaint = true);
cv::Mat borderMask(const cv::Mat& image);
void inpaintBorder(const cv::Mat& input, cv::Mat& output, cv::Mat& mask);

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_padding_h__
