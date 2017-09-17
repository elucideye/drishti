/*! -*-c++-*-
  @file   face_util.h
  @author David Hirvonen
  @brief  Internal declaration for some simple face model utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_face_util_h__
#define __drishti_face_face_util_h__

#include "drishti/face/drishti_face.h"
#include "drishti/face/Face.h"

#include <opencv2/videostab.hpp> // TODO replace cv::videostab::MotionModel type with int?

#include <vector>
#include <stdio.h>

DRISHTI_FACE_NAMESPACE_BEGIN

std::vector<float> faceToVector(const FaceModel& face, bool crease = false, bool brow = false, bool nose = false);
cv::Mat estimateGlobalMotionLeastSquares(const FaceModel& src, const FaceModel& dst, cv::videostab::MotionModel model);
cv::Point2f estimateGaze(const DRISHTI_FACE::FaceModel& face);

DRISHTI_FACE_NAMESPACE_END

#endif /* defined(__drishti_face_face_util_h__) */
