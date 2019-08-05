/*! -*-c++-*-
  @file   face_util.cpp
  @author David Hirvonen
  @brief  Internal implementation for some simple face model utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/face_util.h"
#include "drishti/face/Face.h"
#include "drishti/eye/EyeIO.h"
#include "drishti/geometry/Primitives.h"

#include <array>

DRISHTI_FACE_NAMESPACE_BEGIN

using PointVec = std::vector<cv::Point2f>;

std::vector<float> faceToVector(const FaceModel& face, bool crease, bool brow, bool nose)
{
    std::vector<float> params;
    if (face.eyeFullR.has && face.eyeFullL.has)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::eyeToVector(*face.eyeFullR, crease));
        DRISHTI_EYE::cat(params, DRISHTI_EYE::eyeToVector(*face.eyeFullL, crease));
    }

    if (brow)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.eyebrowRight));
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.eyebrowLeft));
    }
    if (nose)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.nose));
    }
    return params;
}

DRISHTI_FACE_NAMESPACE_END
