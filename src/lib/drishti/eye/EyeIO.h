/*! -*-c++-*-
  @file   EyeIO.h
  @author David Hirvonen
  @brief  Declaration of internal routines related to eye model conversion.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains declarations of various routines related to instantiating
  and converting eye model structures.

*/

#ifndef __drishti_eye_EyeIO_h__
#define __drishti_eye_EyeIO_h__ 1

#include "drishti/eye/drishti_eye.h"
#include "drishti/eye/Eye.h"
#include "drishti/core/Shape.h"

DRISHTI_EYE_NAMESPACE_BEGIN

struct EyeModelSpecification
{
    static EyeModelSpecification create(
        int eyelidCount = 16,
        int creaseCount = 9,
        bool irisCenter = true,
        bool irisOuter = true,
        bool irisInner = true,
        bool irisEllipse = true,
        bool pupilEllipse = true);

    template <class Archive>
    void serialize(Archive& ar, const uint32_t version);

    cv::Range eyelids;
    cv::Range crease;
    cv::Range irisCenter;
    cv::Range irisOuter;
    cv::Range irisInner;
    cv::Range irisEllipse;
    cv::Range pupilEllipse;
};

EyeModel shapeToEye(const std::vector<cv::Point2f>& points, const EyeModelSpecification& spec);

std::vector<cv::Point2f> eyeToShape(const EyeModel& eye, const EyeModelSpecification& spec);
std::vector<float> eyeToVector(const EyeModel& eye, bool crease = true);
std::vector<float> pointsToVector(const std::vector<cv::Point2f>& points);
std::vector<float>& cat(std::vector<float>& src, const std::vector<float>& params);

DRISHTI_EYE_NAMESPACE_END

#endif // __drishti_eye_EyeIO_h__
