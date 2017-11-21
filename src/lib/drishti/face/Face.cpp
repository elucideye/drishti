/*! -*-c++-*-
  @file   Face.cpp
  @author David Hirvonen
  @brief  Internal implementation of a utility face model class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/Face.h"
#include "drishti/core/Shape.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/geometry/motion.h"
#include "drishti/face/FaceIO.h"

#include <opencv2/videostab/global_motion.hpp>
#include <opencv2/video/tracking.hpp>
#include <opencv2/highgui.hpp>

#include <array>

DRISHTI_FACE_NAMESPACE_BEGIN

FaceModel::FaceModel() {} //  = default;
FaceModel::~FaceModel() = default;

float FaceModel::getInterPupillaryDistance() const
{
    float ipd = 0.f;
    if (eyeFullL.has && eyeFullR.has)
    {
        ipd = cv::norm(eyeFullL->irisEllipse.center - eyeFullR->irisEllipse.center);
    }
    return ipd;
}

bool FaceModel::getEyeRegions(cv::Rect2f& eyeR, cv::Rect2f& eyeL, float scale) const
{
    bool okay = false;
    if ((eyeLeft.size() && eyeRight.size()) || (eyeRightCenter.has && eyeLeftCenter.has))
    {
        okay = true;
        cv::Point2f pL = getEyeLeftCenter();
        cv::Point2f pR = getEyeRightCenter();
        cv::Point2f ratio(1.0, 3.0 / 4.0);
        cv::Point2f diag = ratio * cv::norm(pL - pR) * scale * 0.5f;
        eyeL = cv::Rect2f((pL - diag), (pL + diag));
        eyeR = cv::Rect2f((pR - diag), (pR + diag));
    }
    return okay;
}

cv::Point2f FaceModel::getEyeLeftCenter() const
{
    return eyeLeft.size() ? drishti::core::centroid(eyeLeft) : (eyeLeftCenter.has ? eyeLeftCenter.value : cv::Point2f(0, 0));
}

cv::Point2f FaceModel::getEyeRightCenter() const
{
    return eyeRight.size() ? drishti::core::centroid(eyeRight) : (eyeRightCenter.has ? eyeRightCenter.value : cv::Point2f(0, 0));
}

std::vector<cv::Point2f> asSpline(const std::vector<cv::Point2f>& points, int n, bool closed)
{
    std::vector<cv::Point2f> spline;
    drishti::core::fitSpline(points, spline, n, closed);
    if (closed)
    {
        spline.push_back(spline.front());
    }

    return spline;
}

std::vector<FaceModel::ContourVec> FaceModel::getFaceParts(bool fullEyes, bool browClosed) const
{
    bool useFull = (fullEyes && eyeFullR.has && eyeFullL.has);
    ContourVec eyeRight_ = useFull ? eyeFullR->getContours() : ContourVec(1, asSpline(eyeRight, 64, true));
    ContourVec eyeLeft_ = useFull ? eyeFullL->getContours() : ContourVec(1, asSpline(eyeLeft, 64, true));

    // clang-format off
    std::vector<ContourVec> features
    {
        { eyeRight_ },
        { eyeLeft_ },
        { asSpline(nose, 64, false) },
        { asSpline(eyebrowRight, 64, browClosed) },
        { asSpline(eyebrowLeft, 64, browClosed) }
    };
    
    if(mouthOuter.size() && mouthInner.size())
    {
        features.emplace_back( ContourVec{asSpline(mouthOuter, 64, true)} );
        features.emplace_back( ContourVec{asSpline(mouthInner, 64, true)} );
    }
    // clang-format on

    if (sideLeft.size() && sideRight.size())
    {
        auto cL = asSpline(sideLeft, 10, false);
        auto cR = asSpline(sideRight, 10, false);

        features.push_back({ cL });
        features.push_back({ cR });
    }

    return features;
};

void FaceModel::draw(cv::Mat& canvas, int width, bool fullEyes, bool allPoints) const
{
    static std::vector<cv::Vec3b> rainbow{
        { 0, 0, 255 },     // red
        { 0, 127, 255 },   // orange
        { 0, 200, 200 },   // yellow
        { 0, 255, 0 },     // green
        { 255, 0, 0 },     // blue
        { 130, 0, 75 },    // indigo
        { 255, 0, 139 },   // violet
        { 127, 127, 127 }, // white
    };

    if (allPoints && points.has)
    {
        for (auto& p : *points)
        {
            cv::circle(canvas, p, width, { 0, 255, 0 }, -1, 8);
        }
    }

    const bool browClosed = false;
    auto features = getFaceParts(fullEyes, browClosed);

    // Draw the nose tip estimate:
    std::vector<const core::Field<cv::Point2f>*> things{
        &noseTip,
        &eyebrowLeftInner,
        &eyebrowRightInner,
        &mouthCornerLeft,
        &mouthCornerRight
    };

    if (!fullEyes)
    {
        things.push_back(&eyeLeftInner);
        things.push_back(&eyeLeftOuter);
        things.push_back(&eyeRightInner);
        things.push_back(&eyeRightOuter);
    }

    // ### Do the actual drawing ###
    cv::rectangle(canvas, roi, { 0, 255, 0 }, width, 8);
    for (int i = 0; i < features.size(); i++)
    {
        auto& f = features[i];
        if (f.size())
        {
            std::vector<std::vector<cv::Point>> points(f.size());
            for (int j = 0; j < f.size(); j++)
            {
                std::copy(f[j].begin(), f[j].end(), std::back_inserter(points[j]));
            }
            cv::polylines(canvas, points, false, rainbow[i % rainbow.size()], width);
        }
    }

    for (auto& p : things)
    {
        if (p->has)
        {
            cv::circle(canvas, p->value, 4, { 0, 255, 0 }, -1, 8);
        }
    }
}

cv::Mat getSimilarityMotionFromEyes(const FaceModel& a, const FaceModel& b)
{
    std::array<cv::Point2f, 2> ptsA{ { a.eyeFullR->irisEllipse.center, a.eyeFullL->irisEllipse.center } };
    std::array<cv::Point2f, 2> ptsB{ { b.getEyeRightCenter(), b.getEyeLeftCenter() } };
    //std::array<cv::Point2f,2> ptsB {{ b.eyeFullR->irisEllipse.center, b.eyeFullL->irisEllipse.center }};

    return cv::Mat(transformation::estimateSimilarity(ptsA, ptsB));
}

cv::Mat getSimilarityMotion(const FaceModel& a, const FaceModel& b)
{
    std::array<cv::Point2f, 2> ptsA{ { a.getEyeRightCenter(), a.getEyeLeftCenter() } };
    std::array<cv::Point2f, 2> ptsB{ { b.getEyeRightCenter(), b.getEyeLeftCenter() } };
    return cv::Mat(transformation::estimateSimilarity(ptsA, ptsB));
}

// Use eyes + nose tip, since this is compatible with both full and inner face landmark sets:
cv::Mat getAffineMotion(const FaceModel& a, const FaceModel& b)
{
    CV_Assert(a.eyeLeftCenter.has);
    CV_Assert(a.eyeRightCenter.has);
    CV_Assert(a.noseTip.has);
    cv::Point2f ptsA[3] = { a.eyeRightCenter.value, a.eyeLeftCenter.value, a.noseTip.value };

    CV_Assert(b.eyeLeftCenter.has);
    CV_Assert(b.eyeRightCenter.has);
    CV_Assert(b.noseTip.has);
    cv::Point2f ptsB[3] = { b.eyeRightCenter.value, b.eyeLeftCenter.value, b.noseTip };

    cv::Mat H = cv::getAffineTransform(&ptsA[0], &ptsB[0]);
    return H;
}

// Use eyes + nose tip, since this is compatible with both full and inner face landmark sets:
cv::Mat estimateMotionLeastSquares(const FaceModel& a, const FaceModel& b)
{
    CV_Assert(a.eyeLeftCenter.has);
    CV_Assert(a.eyeRightCenter.has);
    CV_Assert(a.noseTip.has);
    std::vector<cv::Point2f> ptsA{ a.eyeRightCenter.value, a.eyeLeftCenter.value, a.noseTip.value };

    CV_Assert(b.eyeLeftCenter.has);
    CV_Assert(b.eyeRightCenter.has);
    CV_Assert(b.noseTip.has);
    std::vector<cv::Point2f> ptsB{ b.eyeRightCenter.value, b.eyeLeftCenter.value, b.noseTip.value };

    cv::Mat H = cv::videostab::estimateGlobalMotionLeastSquares(ptsA, ptsB, cv::videostab::MM_SIMILARITY);
    if (!H.empty())
    {
        H.convertTo(H, CV_64F);
    }
    return H;
}

DRISHTI_FACE_NAMESPACE_END
