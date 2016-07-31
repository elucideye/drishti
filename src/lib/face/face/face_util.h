/*!
  @file   face_util.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Internal declaration for some simple face model utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__face_util__
#define __drishtisdk__face_util__

#include "face/drishti_face.h"
#include "face/Face.h"

#include <opencv2/videostab.hpp> // TODO replace cv::videostab::MotionModel type with int?

#include <vector>
#include <stdio.h>

BEGIN_FACE_NAMESPACE

std::vector<float> faceToVector(const FaceModel &face, bool crease=false, bool brow=false, bool nose=false);
cv::Mat estimateGlobalMotionLeastSquares(const FaceModel &src, const FaceModel &dst, cv::videostab::MotionModel model);
cv::Point2f estimateGaze(const DRISHTI_FACE::FaceModel &face);

END_FACE_NAMESPACE


#endif /* defined(__drishtisdk__face_util__) */
