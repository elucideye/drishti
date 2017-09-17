/*! -*-c++-*-
  @file   EyeIO.cpp
  @author David Hirvonen
  @brief  Implemenation of internal routines related to eye model conversion.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains implementations of various routines related to instantiating
  and converting eye model structures.

*/

#include "drishti/eye/EyeIO.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/core/drishti_stdlib_string.h" // ANDROID
#include "drishti/core/drishti_cv_cereal.h"

#include <cereal/archives/json.hpp>

DRISHTI_EYE_NAMESPACE_BEGIN

std::vector<float>& cat(std::vector<float>& src, const std::vector<float>& params)
{
    std::copy(params.begin(), params.end(), std::back_inserter(src));
    return src;
}

static std::vector<float>& addPoint(std::vector<float>& params, const cv::Point2f& p)
{
    params.push_back(p.x);
    params.push_back(p.y);
    return params;
}

std::vector<float> pointsToVector(const std::vector<cv::Point2f>& points)
{
    std::vector<float> params;
    for (const auto& p : points)
    {
        addPoint(params, p);
    }
    return params;
}

static std::vector<float>& addPoints(std::vector<float>& params, const std::vector<cv::Point2f>& points)
{
    std::vector<float> tmp = pointsToVector(points);
    std::copy(tmp.begin(), tmp.end(), std::back_inserter(params));
    return params;
}

static std::vector<float>& addEllipse(std::vector<float>& params, const cv::RotatedRect& ellipse)
{
    std::vector<float> phi = drishti::geometry::ellipseToPhi(ellipse);
    std::copy(phi.begin(), phi.end(), std::back_inserter(params));
    return params;
}

std::vector<float> eyeToVector(const EyeModel& eye, bool crease)
{
    std::vector<float> params;
    addPoints(params, eye.eyelids);
    addPoint(params, eye.irisCenter);
    addPoint(params, eye.irisInner);
    addPoint(params, eye.irisOuter);
    addEllipse(params, eye.irisEllipse);
    addEllipse(params, eye.pupilEllipse);
    if (crease)
    {
        addPoints(params, eye.crease);
    }
    return params;
}

static cv::Range add(int count, const cv::Range& last = { 0, 0 })
{
    return cv::Range(last.end, last.end + count);
}

EyeModelSpecification EyeModelSpecification::create(
    int eyelidCount,
    int creaseCount,
    bool irisCenter,
    bool irisOuter,
    bool irisInner,
    bool irisEllipse,
    bool pupilEllipse)
{
    EyeModelSpecification spec;
    spec.eyelids = add(eyelidCount);
    spec.crease = add(creaseCount, spec.eyelids);
    spec.irisCenter = add(int(irisCenter), spec.crease);
    spec.irisOuter = add(int(irisOuter), spec.irisCenter);
    spec.irisInner = add(int(irisInner), spec.irisOuter);
    spec.irisEllipse = add(int(irisEllipse) * 5, spec.irisInner);     // *
    spec.pupilEllipse = add(int(pupilEllipse) * 5, spec.irisEllipse); // *
    // (*) : Note, ellipse models must come at the end for current regression code

    return spec;
}

static void fill(const std::vector<cv::Point2f>& points, const cv::Range& range, std::vector<cv::Point2f>& output)
{
    output.resize(range.end - range.start);
    for (int i = 0; i < output.size(); i++)
    {
        output[i] = points[i + range.start];
        CV_Assert(i < points.size());
    }
}

static void fill(const std::vector<cv::Point2f>& points, const cv::Range& range, core::Field<cv::Point2f>& output)
{
    if (range.end != 0)
    {
        CV_Assert((range.end - range.start) == 1);
        output = points[range.start];
        CV_Assert(range.start < points.size());
    }
}

using PointVec = std::vector<cv::Point2f>;
static void fill(const std::vector<cv::Point2f>& points, const cv::Range& range, cv::RotatedRect& output)
{
    if (range.end != 0)
    {
        CV_Assert((range.end - range.start) == 5 && (range.end <= points.size()));
        PointVec slice(points.begin() + range.start, points.begin() + range.end);
        output = geometry::pointsToEllipse(slice);
    }
}

EyeModel shapeToEye(const std::vector<cv::Point2f>& points, const EyeModelSpecification& spec)
{
    EyeModel eye;
    fill(points, spec.eyelids, eye.eyelids);
    fill(points, spec.crease, eye.crease);
    fill(points, spec.irisCenter, eye.irisCenter);
    fill(points, spec.irisOuter, eye.irisOuter);
    fill(points, spec.irisInner, eye.irisInner);
    fill(points, spec.irisEllipse, eye.irisEllipse);   // *
    fill(points, spec.pupilEllipse, eye.pupilEllipse); // *
    // (*) : Note, ellipse models must come at the end for current regression code

    CV_Assert(spec.pupilEllipse.end == points.size());

    eye.cornerIndices[0] = 0;
    eye.cornerIndices[1] = spec.eyelids.end / 2;

    // Convenience:
    eye.innerCorner = eye.getInnerCorner();
    eye.outerCorner = eye.getOuterCorner();

    eye.refine();
    eye.roi = cv::boundingRect(eye.eyelids);

    return eye;
}

static void copy(std::vector<cv::Point2f>& points, const cv::Range& range, const std::vector<cv::Point2f>& input)
{
    std::copy(input.begin(), input.end(), std::back_inserter(points));
}

static void copy(std::vector<cv::Point2f>& points, const cv::Range& range, const core::Field<cv::Point2f>& input)
{
    if (input.has)
    {
        points.push_back(input.get());
    }
}

static void copy(std::vector<cv::Point2f>& points, const cv::Range& range, const cv::RotatedRect& input)
{
    auto output = geometry::ellipseToPoints(input);
    output.back().x *= (M_PI / 180.0);
    std::copy(output.begin(), output.end(), std::back_inserter(points));
}

std::vector<cv::Point2f> eyeToShape(const EyeModel& eye, const EyeModelSpecification& spec)
{
    std::vector<cv::Point2f> points;
    copy(points, spec.eyelids, eye.eyelids);
    copy(points, spec.crease, eye.crease);
    copy(points, spec.irisCenter, eye.irisCenter);
    copy(points, spec.irisOuter, eye.irisOuter);
    copy(points, spec.irisInner, eye.irisInner);
    copy(points, spec.irisEllipse, eye.irisEllipse);   // *
    copy(points, spec.pupilEllipse, eye.pupilEllipse); // *
    // (*) : Note, ellipse models must come at the end for current regression code

    return points;
}

template <class Archive>
void EyeModelSpecification::serialize(Archive& ar, const unsigned int version)
{
    ar& GENERIC_NVP("eyelids", eyelids);
    ar& GENERIC_NVP("crease", crease);
    ar& GENERIC_NVP("irisCenter", irisCenter);
    ar& GENERIC_NVP("irisOuter", irisOuter);
    ar& GENERIC_NVP("irisInner", irisInner);
    ar& GENERIC_NVP("irisEllipse", irisEllipse);
    ar& GENERIC_NVP("pupilEllipse", pupilEllipse);
}

typedef cereal::JSONOutputArchive OArchiveJSON;
template void EyeModelSpecification::serialize<OArchiveJSON>(OArchiveJSON& ar, const unsigned int);

typedef cereal::JSONInputArchive IArchiveJSON;
template void EyeModelSpecification::serialize<IArchiveJSON>(IArchiveJSON& ar, const unsigned int);

DRISHTI_EYE_NAMESPACE_END
