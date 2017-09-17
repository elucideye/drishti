/*! -*-c++-*-
  @file   Eye.cpp
  @author David Hirvonen
  @brief  Top level API eye model implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the eye model structure used
  to report results for the top level SDK.
*/

#include "drishti/Eye.hpp"
#include "drishti/drishti_cv.hpp"
#include "drishti/eye/Eye.h"

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <iostream>
#include <sstream>
#include <cmath>

_DRISHTI_SDK_BEGIN

/*
 * Eye
 */

Eye::Eye()
{
}

Eye::Eye(const Eye& src)
    : iris(src.iris)
    , pupil(src.pupil)
    , eyelids(src.eyelids)
    , crease(src.crease)
    , innerCorner(src.innerCorner)
    , outerCorner(src.outerCorner)
    , roi(src.roi)
{
}

void createMask(Image1b& mask, const Eye& eye, int components)
{
    if (mask.getRows() && mask.getCols() && eye.getEyelids().size())
    {
        // Convert eyelids to opencv contour:
        auto eyelids = drishtiToCv(eye.getEyelids());
        std::vector<std::vector<cv::Point>> contours(1);
        std::copy(eyelids.begin(), eyelids.end(), std::back_inserter(contours[0]));

        cv::Mat maskHandle = drishtiToCv<uint8_t, uint8_t>(mask); // wrapper (shallow copy)
        if ((components & kScleraRegion) && (components & kIrisRegion) && (components & kPupilRegion))
        {
            cv::fillPoly(maskHandle, contours, 255, 4);
            return;
        }

        cv::Mat1b eyeMask(mask.getRows(), mask.getCols(), uint8_t(0));
        cv::fillPoly(eyeMask, contours, 2550, 4);
        if (components & kScleraRegion)
        {
            maskHandle.setTo(255, eyeMask);
        }
        if (eye.getIris().size.width > 0.f && eye.getIris().size.height > 0.f)
        {
            cv::ellipse(maskHandle, drishtiToCv(eye.getIris()), (components & kIrisRegion) ? 255 : 0, -1, 4);
        }
        if (eye.getPupil().size.width > 0.f && eye.getPupil().size.height > 0.f)
        {
            cv::ellipse(maskHandle, drishtiToCv(eye.getPupil()), (components & kPupilRegion) ? 255 : 0, -1, 4);
        }
        maskHandle.setTo(0, ~eyeMask);
    }
}

_DRISHTI_SDK_END
