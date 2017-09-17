/*! -*-c++-*-
  @file   Eye.h
  @author David Hirvonen
  @brief  Internal utility eye model class declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the internal SDK eye model,
  which contains extended functionality compared to the public SDK eye model.

*/

#ifndef __drishti_eye_Eye_h__
#define __drishti_eye_Eye_h__

#include "drishti/eye/drishti_eye.h"
#include "drishti/core/Field.h"
#include "drishti/geometry/Rectangle.h"

#define DRISHTI_EYE_CONTOUR_POINTS 64
#define DRISHTI_EYE_CREASE_POINTS 16

DRISHTI_EYE_NAMESPACE_BEGIN

using drishti::geometry::operator*;

DRISHTI_BEGIN_NAMESPACE(circle)
inline cv::Point2f center(const cv::Vec3f& circle)
{
    return cv::Point2f(circle[0], circle[1]);
}
inline float radius(const cv::Vec3f& circle)
{
    return circle[2];
}
DRISHTI_END_NAMESPACE(circle)

struct EyeModel
{
    EyeModel();
    ~EyeModel();

    const cv::Point2f& getOuterCorner() const
    {
        return eyelids[cornerIndices[0]];
    }
    const cv::Point2f& getInnerCorner() const
    {
        return eyelids[cornerIndices[1]];
    }

    std::vector<cv::Point2f> getUpperEyelid() const;
    std::vector<cv::Point2f> getLowerEyelid() const;

    cv::Point2f& getOuterCorner()
    {
        return eyelids[cornerIndices[0]];
    }
    cv::Point2f& getInnerCorner()
    {
        return eyelids[cornerIndices[1]];
    }

    const cv::Point2f getPupilCenter() const
    {
        return circle::center(pupil);
    }

    float openness() const;
    cv::Point2f estimateGaze(bool isRight = true) const;
    void estimateIrisLandmarks(cv::Point2f& irisCenter, cv::Point2f& irisInner, cv::Point2f& irisOuter) const;

    void draw(cv::Mat& canvas, int level = 0, bool doMask = true, const cv::Scalar& color = { 0., 255., 0. }, int width = 2) const;
    cv::Mat mask(const cv::Size& size, bool sclera = true, float irisScale = 1.f) const;
    cv::Mat labels(const cv::Size& size) const;
    cv::Mat irisMask(const cv::Size& size, bool removeEyelids = true) const;

    // Support line drawing/contours (OpenGL friendly)
    std::vector<std::vector<cv::Point2f>> getContours(bool doPupil = true) const;

    static void normalizeEllipse(cv::RotatedRect& e);
    void normalize();
    void refine(int eyelidPoints = DRISHTI_EYE_CONTOUR_POINTS, int creasePoints = DRISHTI_EYE_CREASE_POINTS);
    void upsample(int eyelidFactor = 2, int creaseFactor = 2);
    void clear();

    static void flop(float& x, float width)
    {
        x = (width - x);
    }

    void flop(int width);

    template <typename T>
    EyeModel& operator+=(const cv::Point_<T>& p)
    {
        *this = *this + p;
        return *this;
    }
    template <typename T>
    EyeModel& operator-=(const cv::Point_<T>& p)
    {
        *this = *this - p;
        return *this;
    }

    template <typename T>
    EyeModel& operator*=(const T& s)
    {
        *this = *this * s;
        return *this;
    }

    // ### IO

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

    void write(const std::string& filename) const;
    void read(const std::string& filename);

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    // ### data

    core::Field<float> angle; // radians

    core::Field<cv::Rect> roi;
    cv::Vec3f pupil;
    cv::Vec3f iris;
    cv::RotatedRect irisEllipse;
    cv::RotatedRect pupilEllipse;

    int cornerIndices[2] = { 0, 8 }; // assuming 16 point contour
    std::vector<cv::Point2f> eyelids;
    std::vector<cv::Point2f> eyelidsSpline;

    core::Field<cv::Point2f> innerCorner;
    core::Field<cv::Point2f> outerCorner;

    std::vector<cv::Point2f> crease;
    std::vector<cv::Point2f> creaseSpline;

    // 3-point iris estimate:
    core::Field<cv::Point2f> irisCenter;
    core::Field<cv::Point2f> irisInner;
    core::Field<cv::Point2f> irisOuter;
};

inline std::vector<std::vector<cv::Point2f>*> getEyeModelContours(EyeModel& dst)
{
    return std::vector<std::vector<cv::Point2f>*>{
        &dst.eyelids,
        &dst.eyelidsSpline,
        &dst.crease,
        &dst.creaseSpline
    };
}

inline std::vector<cv::Point2f*> getEyeModelLandmarks(EyeModel& dst)
{
    return std::vector<cv::Point2f*>{
        &dst.irisCenter.value,
        &dst.irisInner.value,
        &dst.irisOuter.value
    };
}

inline std::vector<cv::RotatedRect*> getEyeModelEllipses(EyeModel& dst)
{
    return std::vector<cv::RotatedRect*>{
        &dst.irisEllipse,
        &dst.pupilEllipse
    };
}

inline std::vector<cv::Vec3f*> getEyeModelCircles(EyeModel& dst)
{
    return std::vector<cv::Vec3f*>{
        &dst.pupil,
        &dst.iris
    };
}

inline bool operator==(const cv::RotatedRect& a, const cv::RotatedRect& b)
{
    return (a.center == b.center) && (a.size == b.size) && (a.angle == b.angle);
}

inline bool operator==(const std::vector<cv::Point2f>& a, const std::vector<cv::Point2f>& b)
{
    if (a.size() != b.size())
    {
        return false;
    }
    for (int i = 0; i < a.size(); i++)
    {
        if (a[i] != b[i])
        {
            return false;
        }
    }
    return true;
}

inline bool operator==(const core::Field<cv::Point2f>& a, const core::Field<cv::Point2f>& b)
{
    return (a.has && b.has) && (*a == *b);
}

inline bool operator==(const EyeModel& a, const EyeModel& b)
{
    return (a.eyelids == b.eyelids) &&                        // contours
        (a.eyelidsSpline == b.eyelidsSpline) &&
        (a.crease == b.crease) &&
        (a.creaseSpline == b.creaseSpline) &&
        (a.irisCenter == b.irisCenter) &&
        (a.irisInner == b.irisInner) &&                       // landmarks
        (a.irisOuter == b.irisOuter) &&
        (a.irisEllipse == b.irisEllipse) &&                   // ellipse models
        (a.pupilEllipse == b.pupilEllipse) &&
        (a.iris == b.iris) &&                                 // circle models
        (a.pupil == b.pupil);
}

template <typename T>
EyeModel operator*(const cv::Matx<T, 3, 3>& H, const EyeModel& src)
{
    auto dst = src;

    if (dst.roi.has)
    {
        dst.roi = H * *dst.roi;
    }

    if (dst.irisCenter.has)
    {
        auto landmarks = getEyeModelLandmarks(dst);
        for (const auto& p : landmarks)
        {
            cv::Point3f q = H * cv::Point3f(p->x, p->y, 1.f);
            *p = { q.x / q.z, q.y / q.z };
        }
    }

    auto contours = getEyeModelContours(dst);
    for (auto& c : contours)
    {
        if (c->size())
        {
            for (auto& p : *c)
            {
                cv::Point3f q = H * cv::Point3f(p.x, p.y, 1.f);
                p = { q.x / q.z, q.y / q.z };
            }
        }
    }

    auto ellipses = getEyeModelEllipses(dst);
    for (auto& e : ellipses)
    {
        if (e->size.area())
        {
            *e = H * *e;
        }
    }

    auto circles = getEyeModelCircles(dst);
    for (auto& c : circles)
    {
        if ((*c)[2])
        {
            *c = H * *c;
        }
    }

    return dst;
}

template <typename T>
EyeModel operator*(const EyeModel& src, T scale)
{
    cv::Matx33f H(cv::Matx33f::diag({ scale, scale, 1 }));
    EyeModel dst = H * src;
    return dst;
}

template <typename T>
EyeModel operator+(const EyeModel& src, const cv::Point_<T>& offset)
{
    cv::Matx33f H(1, 0, offset.x, 0, 1, offset.y, 0, 0, 1);
    EyeModel dst = H * src;
    return dst;
}

template <typename T>
EyeModel operator-(const EyeModel& src, const cv::Point_<T>& offset)
{
    return src + (-offset);
}

void write(cv::FileStorage& fs, const std::string&, const EyeModel& x);
void read(const cv::FileNode& node, EyeModel& x, const EyeModel& default_value = {});

DRISHTI_EYE_NAMESPACE_END

#endif
