/*!
  @file   IrisNormalizer.h
  @author David Hirvonen
  @brief  Declaration of internal class for creating ellipso-polar normalized iris images.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__IrisNormalizer__
#define __drishtisdk__IrisNormalizer__

#include <stdio.h>

#include "eye/drishti_eye.h"
#include "eye/Eye.h"
#include "eye/NormalizedIris.h"

#include <array>

DRISHTI_EYE_BEGIN

class IrisNormalizer
{
public:

    using Ray=std::array<cv::Point2f, 2>;
    using Rays=std::vector<Ray>;

    IrisNormalizer();

    cv::Size createRays(const EyeModel &eye, const cv::Size &size, Rays &rayPixels, Rays &rayTexels, int padding=0) const;
    void warpIris(const cv::Mat &crop, const cv::Mat1b &mask, const cv::Size &paddedSize, Rays &rayPixels, Rays &rayTexels, NormalizedIris &code, int padding=0) const;
    void operator()(const cv::Mat &crop, const EyeModel &eye, const cv::Size &size, NormalizedIris &code, int padding=0) const;
protected:
};

struct CurveNormalizer
{
public:
    CurveNormalizer();
    void operator()(const cv::Mat &crop, const std::vector<cv::Point2f> &curve, const cv::Size &size, NormalizedCurve &code) const;
protected:
};

DRISHTI_EYE_END

#endif /* defined(__drishtisdk__IrisNormalizer__) */
