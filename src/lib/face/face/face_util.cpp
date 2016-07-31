/*!
  @file   face_util.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Internal implementation for some simple face model utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "face/face_util.h"
#include "face/Face.h"
#include "eye/EyeIO.h"
#include "geometry/Primitives.h"

#include <opencv2/videostab/global_motion.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>

#include <array>

BEGIN_FACE_NAMESPACE

using PointVec = std::vector<cv::Point2f>;

static PointVec& cat(PointVec &points, const PointVec &src)
{
    std::copy(src.begin(), src.end(), std::back_inserter(points));
    return points;
}

cv::Mat estimateGlobalMotionLeastSquares(const FaceModel &a, const FaceModel &b, cv::videostab::MotionModel model)
{
    CV_Assert(a.eyeLeftCenter.has);
    CV_Assert(a.eyeRightCenter.has);
    CV_Assert(a.noseTip.has);
    PointVec ptsA { a.eyeRightCenter, a.eyeLeftCenter, a.noseTip };
    if(a.mouthCornerLeft.has && a.mouthCornerRight.has)
    {
        ptsA.push_back(a.mouthCornerLeft);
        ptsA.push_back(a.mouthCornerRight);
    }

    CV_Assert(b.eyeLeftCenter.has);
    CV_Assert(b.eyeRightCenter.has);
    CV_Assert(b.noseTip.has);
    PointVec ptsB { b.eyeRightCenter, b.eyeLeftCenter, b.noseTip };
    if(b.mouthCornerLeft.has && b.mouthCornerRight.has)
    {
        ptsB.push_back(b.mouthCornerLeft);
        ptsB.push_back(b.mouthCornerRight);
    }

    float rmse = 0;
    cv::Mat M = cv::videostab::estimateGlobalMotionLeastSquares(ptsA, ptsB, model, &rmse);

    return M;
}

std::vector<float> faceToVector(const FaceModel &face, bool crease, bool brow, bool nose)
{
    std::vector<float> params;
    if(face.eyeFullR.has && face.eyeFullL.has)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::eyeToVector(*face.eyeFullR, crease));
        DRISHTI_EYE::cat(params, DRISHTI_EYE::eyeToVector(*face.eyeFullL, crease));
    }

    if(brow)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.eyebrowRight));
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.eyebrowLeft));
    }
    if(nose)
    {
        DRISHTI_EYE::cat(params, DRISHTI_EYE::pointsToVector(face.nose));
    }
    return params;
}

END_FACE_NAMESPACE
