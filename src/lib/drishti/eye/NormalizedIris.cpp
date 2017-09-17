/*! -*-c++-*-
  @file   NormalizedIris.cpp
  @author David Hirvonen
  @brief  Implementation of ellipso-polar iris normalization.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/NormalizedIris.h"

#include <opencv2/imgproc.hpp>

DRISHTI_EYE_NAMESPACE_BEGIN

// Example:
//
// 23456789abcdef1 (-1)
// 123456789abcdef
// f123456789abcde (+1)
//
// Note: (-n) % k == k - (n % k)

void NormalizedIris::rotate(const cv::Mat& image, cv::Mat& dest, int x)
{
    dest.create(image.size(), image.type());
    if (x == 0)
    {
        image.copyTo(dest);
    }
    else
    {
        int rem = (x < 0) ? (image.cols - (-x % image.cols)) : (x % image.cols);
        image(cv::Range::all(), { image.cols - rem, image.cols }).copyTo(dest(cv::Range::all(), { 0, rem }));
        image(cv::Range::all(), { 0, image.cols - rem }).copyTo(dest(cv::Range::all(), { rem, image.cols }));
    }
}

NormalizedIris NormalizedIris::rotate(int x) const
{
    NormalizedIris code;
    code.roi = { { 0, 0 }, roi.size() };
    rotate(getImage(), code.getPaddedImage(), x);
    rotate(getMask(), code.getPaddedMask(), x);
    return code;
}

DRISHTI_EYE_NAMESPACE_END
