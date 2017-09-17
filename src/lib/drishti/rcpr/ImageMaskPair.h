/*! -*-c++-*-
  @file   ImageMaskPair.h
  @author David Hirvonen
  @brief  Declaration of utility image + mask container.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_ImageMaskPair_h__
#define __drishti_rcpr_ImageMaskPair_h__ 1

#include "drishti/rcpr/drishti_rcpr.h"

DRISHTI_RCPR_NAMESPACE_BEGIN

struct ImageMaskPair
{
public:
    ImageMaskPair() {}
    ImageMaskPair(const cv::Mat& image)
        : image(image)
    {
    }
    ImageMaskPair(const cv::Mat& image, const cv::Mat& mask)
        : image(image)
        , mask(mask)
    {
    }

    const cv::Mat& getImage() const
    {
        return image;
    }
    cv::Mat& getImage()
    {
        return image;
    }
    const cv::Mat& getMask() const
    {
        return mask;
    }
    cv::Mat& getMask()
    {
        return mask;
    }

    operator cv::Mat()
    {
        return image; // legacy ImageVec compatibility
    }

protected:
    cv::Mat image;
    cv::Mat mask;
};

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_ImageMaskPair_h__ 1
