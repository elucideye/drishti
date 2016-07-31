/*!
  @file   padding.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of simple aspect ratio aware padding routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti_core.h"
#include <opencv2/core/core.hpp>

#ifndef CORE_PADDING_H
#define CORE_PADDING_H

DRISHTI_CORE_BEGIN

cv::Point padWithInpainting(const cv::Mat &image, cv::Mat &padded, int top, int bottom, int left, int right, bool inPaint=true);
cv::Point padToAspectRatio(const cv::Mat &image, cv::Mat &padded, double aspectRatio, bool inPaint=true);
cv::Point padToWidthUsingAspectRatio(const cv::Mat &canvas, cv::Mat &padded, int width, double aspectRatio, bool inPaint=true);
cv::Mat borderMask(const cv::Mat &image);
void inpaintBorder(const cv::Mat &input, cv::Mat &output, cv::Mat &mask);

DRISHTI_CORE_END

#endif // CORE_PADDING_H
