/*!
  @file   Face.h
  @author David Hirvonen
  @brief  Internal declaration of a utility face model class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__Face__
#define __drishtisdk__Face__

#include "face/drishti_face.h"
#include "core/drishti_serialize.h"
#include "core/Field.h"
#include "eye/Eye.h"
#include "geometry/Rectangle.h"

#include <stdio.h>

BEGIN_FACE_NAMESPACE

using geometry::operator *;

struct FaceModel
{
    using Contour = std::vector<cv::Point2f>;
    using ContourVec = std::vector<Contour>;

    FaceModel();
    FaceModel(const cv::Rect &roi) : roi(roi) {}

    float getInterPupillaryDistance() const;
    cv::Point2f getEyeLeftCenter() const;
    cv::Point2f getEyeRightCenter() const;
    std::vector< ContourVec > getFaceParts(bool fullEyes=false, bool browClosed=true) const;
    bool getEyeRegions(cv::Rect2f &eyeR, cv::Rect2f &eyeL, float scale=0.666f) const;

    // ======================

    // Note: eyeLeftCenter, eyeRightCenter, noseTip used for affine normalization

    core::Field<Contour> points;

    core::Field<cv::Rect> roi;

    core::Field<cv::Point2f> eyeLeftInner;
    core::Field<cv::Point2f> eyeLeftOuter;
    core::Field<cv::Point2f> eyeLeftCenter;
    core::Field<cv::Point2f> eyebrowLeftInner;
    core::Field<cv::Point2f> eyebrowLeftOuter;

    core::Field<cv::Point2f> eyeRightInner;
    core::Field<cv::Point2f> eyeRightOuter;
    core::Field<cv::Point2f> eyeRightCenter;
    core::Field<cv::Point2f> eyebrowRightInner;
    core::Field<cv::Point2f> eyebrowRightOuter;

    core::Field<cv::Point2f> noseTip;
    core::Field<cv::Point2f> noseNostrilLeft;
    core::Field<cv::Point2f> noseNostrilRight;

    core::Field<cv::Point2f> mouthCornerRight;
    core::Field<cv::Point2f> mouthCornerLeft;

    core::Field<DRISHTI_EYE::EyeModel> eyeFullL;
    core::Field<DRISHTI_EYE::EyeModel> eyeFullR;

    // Contours
    std::vector<cv::Point2f> eyeLeft;
    std::vector<cv::Point2f> eyebrowLeft;

    std::vector<cv::Point2f> eyeRight;
    std::vector<cv::Point2f> eyebrowRight;

    std::vector<cv::Point2f> nose;
    std::vector<cv::Point2f> noseFull;

    std::vector<cv::Point2f> mouthOuter, mouth; // parser compatibility
    std::vector<cv::Point2f> mouthInner;

    // Face sides (partial)
    std::vector<cv::Point2f> sideLeft;
    std::vector<cv::Point2f> sideRight;

    std::vector<cv::Rect2d> rois;

    template <typename T> FaceModel& operator +=(const cv::Point_<T> &p)
    {
        *this = *this + p;
        return *this;
    }
    template <typename T> FaceModel& operator -=(const cv::Point_<T> &p)
    {
        *this = *this - p;
        return *this;
    }

    template <typename T> FaceModel& operator *=(const T &s)
    {
        *this = *this * s;
        return *this;
    }

    void draw(cv::Mat &canvas, int width=1, bool fullEyes=false, bool allPoints=false) const;

    // ((( IO )))
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        if(version >= 1)
        {
            ar & GENERIC_NVP("eye-full-left", eyeFullL);
            ar & GENERIC_NVP("eye-full-right", eyeFullR);
        }

        ar & GENERIC_NVP("eye-left-inner", eyeLeftInner);
        ar & GENERIC_NVP("eye-left-center", eyeLeftCenter);
        ar & GENERIC_NVP("eye-left-outer", eyeLeftOuter);
        ar & GENERIC_NVP("eyebrow-left-inner", eyebrowLeftInner);
        ar & GENERIC_NVP("eyebrow-left-inner", eyebrowLeftOuter);

        ar & GENERIC_NVP("eye-right-inner", eyeRightInner);
        ar & GENERIC_NVP("eye-right-center", eyeRightCenter);
        ar & GENERIC_NVP("eye-right-outer", eyeRightOuter);
        ar & GENERIC_NVP("eyebrow-right-inner", eyebrowRightInner);
        ar & GENERIC_NVP("eyebrow-right-inner", eyebrowRightOuter);

        ar & GENERIC_NVP("nose-tip", noseTip);
        ar & GENERIC_NVP("nose-nostril-left", noseNostrilLeft);
        ar & GENERIC_NVP("nose-nostril-right", noseNostrilRight);

        ar & GENERIC_NVP("mouth-corner-left", mouthCornerLeft);
        ar & GENERIC_NVP("mouth-corner-right", mouthCornerRight);

        ar & GENERIC_NVP("mouth", mouth);
        if(version >= 2)
        {
            ar & GENERIC_NVP("mouthOuter", mouthOuter);
            ar & GENERIC_NVP("mouthInner", mouthInner);
        }
        ar & GENERIC_NVP("nose", nose);
        ar & GENERIC_NVP("eye-left", eyeLeft);
        ar & GENERIC_NVP("eyebrow-left", eyebrowLeft);
        ar & GENERIC_NVP("eye-right", eyeRight);
        ar & GENERIC_NVP("eyebrow-right", eyebrowRight);
    }
};

template <typename T>
FaceModel operator *(const cv::Matx<T, 3, 3> &H, const FaceModel & src)
{
    // Transform all feature contours:
    FaceModel dst = src;

    if(dst.points.has)
    {
        for(auto &p : *dst.points)
        {
            cv::Point3f q = H * cv::Point3_<T>(p.x, p.y, 1.f);
            p = cv::Point2f(q.x/q.z, q.y/q.z);
        }
    }

    std::vector< core::Field<cv::Point2f> * > points
    {
        &dst.eyeLeftInner,
        &dst.eyeLeftCenter,
        &dst.eyeLeftOuter,
        &dst.eyebrowLeftInner,
        &dst.eyebrowLeftOuter,
        &dst.eyeRightInner,
        &dst.eyeRightCenter,
        &dst.eyeRightOuter,
        &dst.eyebrowRightInner,
        &dst.eyebrowRightOuter,
        &dst.noseTip,
        &dst.noseNostrilLeft,
        &dst.noseNostrilRight,
        &dst.mouthCornerLeft,
        &dst.mouthCornerRight
    };
    for(auto &p : points)
    {
        if(p->has)
        {
            cv::Point3f q = H * cv::Point3_<T>(p->value.x, p->value.y, 1.f);
            p->value = cv::Point2f(q.x/q.z, q.y/q.z);
        }
    }

    std::vector< std::vector<cv::Point2f> * > features
    {
        &dst.eyeLeft,
        &dst.eyeRight,
        &dst.nose,
        &dst.eyebrowLeft,
        &dst.eyebrowRight,
        &dst.mouthOuter,
        &dst.mouthInner
    };
    for(auto &f : features)
    {
        for(auto &p : *f)
        {
            cv::Point3f q = H * cv::Point3_<T>(p.x, p.y, 1.f);
            p = cv::Point2f(q.x/q.z, q.y/q.z);
        }
    }

    // Transform the roi:
    dst.roi = H * dst.roi.value;

    for(auto &r : dst.rois)
    {
        r = H * r;
    }

    std::vector< core::Field<DRISHTI_EYE::EyeModel>* > eyes { &dst.eyeFullR, &dst.eyeFullL };
    for(auto &e : eyes)
    {
        if(e->has)
        {
            e->value = H * e->value;
        }
    }

    return dst;
}

template <typename AType>
FaceModel operator *(const FaceModel & src, AType scale)
{
    cv::Matx<AType,3,3> S(cv::Matx33f::diag({scale,scale,1}));
    auto dst = S * src;
    return dst;
}

template <typename AType>
FaceModel operator +(const FaceModel & src, const cv::Point_<AType> &offset )
{
    cv::Matx<AType,3,3> T(1,0,offset.x,0,1,offset.y,0,0,1);
    auto dst = T * src;
    return dst;
}

template <typename AType>
FaceModel operator -(const FaceModel &src, const cv::Point_<AType> &offset)
{
    cv::Matx<AType,3,3> T(1,0,-offset.x,0,1,-offset.y,0,0,1);
    auto dst = T * src;
    return dst;
}

// Two eye similarity transformation:
cv::Mat getSimilarityMotion(const FaceModel &a, const FaceModel &b);

cv::Mat getSimilarityMotionFromEyes(const FaceModel &a, const FaceModel &b);

// Two eyes + nose affine transformation:
cv::Mat getAffineMotion(const FaceModel &a, const FaceModel &b);

// Two eyes + nose similarity transformation:
cv::Mat estimateMotionLeastSquares(const FaceModel &a, const FaceModel &b);

END_FACE_NAMESPACE

CEREAL_CLASS_VERSION(DRISHTI_FACE::FaceModel, 2);

#endif /* defined(__drishtisdk__Face__) */
