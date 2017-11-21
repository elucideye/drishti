/*! -*-c++-*-
  @file   FaceIO.cpp
  @author David Hirvonen
  @brief  Implementation of utilities for insantiating face models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceIO.h"
#include "drishti/eye/EyeIO.h"
#include "drishti/core/Shape.h"
#include "drishti/geometry/Primitives.h"

#include <numeric>

#include <opencv2/highgui.hpp>

DRISHTI_FACE_NAMESPACE_BEGIN

// Assumes roughly upright
static cv::Point2f minPointX(std::vector<cv::Point2f>& points)
{
    return *std::min_element(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
}

static cv::Point2f maxPointX(std::vector<cv::Point2f>& points)
{
    return *std::max_element(points.begin(), points.end(), [](const cv::Point2f& a, const cv::Point2f& b) {
        return a.x < b.x;
    });
}

static void fill(FaceModel& face)
{
    // Estimate centers for normalization:
    face.eyeRightCenter = drishti::core::centroid(face.eyeRight);
    face.eyeLeftCenter = drishti::core::centroid(face.eyeLeft);
    face.noseTip = drishti::core::centroid(face.nose);

    face.eyeRightInner = maxPointX(face.eyeRight);
    face.eyeLeftInner = minPointX(face.eyeLeft);

    face.eyeRightOuter = minPointX(face.eyeRight);
    face.eyeLeftOuter = maxPointX(face.eyeRight);

    if (face.mouthOuter.size())
    {
        face.mouthCornerLeft = minPointX(face.mouthOuter);
        face.mouthCornerRight = maxPointX(face.mouthOuter);
    }
}

// Note: use inclusive range
std::vector<int> iota(const cv::Range& range)
{
    std::vector<int> index(range.end - range.start + 1);
    std::iota(index.begin(), index.end(), range.start);
    return index;
}

static void makeHELEN(FaceSpecification& spec)
{
    spec.eyeR = iota({ 134, 153 });
    spec.eyeL = iota({ 114, 133 });
    spec.nose = iota({ 41, 57 });
    spec.browR = iota({ 174, 193 });
    spec.browL = iota({ 154, 173 });
    spec.mouthOuter = iota({ 58, 85 });
}

//table.browR = { 17,18,19,20,21 };
//table.browL = { 22,23,24,25,26 };
//table.eyeR = { 36,37,38,39,40,41 };
//table.eyeL = { 42,43,44,45,46,47 };
//table.nose = { 27,28,29,30,31,32,33,34,35 };
//table.mouth = { 48,49,50,51,52,53,54,55,56,57,58,59 };

static void makeibug68(FaceSpecification& spec)
{
    spec.eyeR = iota({ 36, 41 });
    spec.eyeL = iota({ 42, 47 });
    spec.nose = iota({ 31, 35 });
    spec.browR = iota({ 17, 21 });
    spec.browL = iota({ 22, 26 });
    spec.mouthOuter = iota({ 48, 58 });
    spec.mouthInner = iota({ 60, 67 });
}

static void makeibug68_inner(FaceSpecification& spec)
{
    static const int start = 17;
    spec.eyeR = iota({ 36 - start, 41 - start });
    spec.eyeL = iota({ 42 - start, 47 - start });
    spec.nose = iota({ 31 - start, 35 - start });
    spec.browR = iota({ 17 - start, 21 - start });
    spec.browL = iota({ 22 - start, 26 - start });
}

FaceSpecification FaceSpecification::create(Format format)
{
    FaceSpecification spec;
    switch (format)
    {
        case kHELEN:
            makeHELEN(spec);
            break;
        case kibug68:
            makeibug68(spec);
            break;
        case kibug68_inner:
            makeibug68_inner(spec);
            break;
    }
    return spec;
}

FaceModel shapeToFace(drishti::core::Shape& shape, const FaceSpecification& spec, bool relative = true)
{
    auto points = shape.getPoints();

    FaceModel face;
    face.roi = shape.roi;
    face.points = points;

    std::vector<std::pair<const std::vector<int>*, std::vector<cv::Point2f>*>> objects{
        { &spec.eyeR, &face.eyeRight },
        { &spec.eyeL, &face.eyeLeft },
        { &spec.nose, &face.nose },
        { &spec.browR, &face.eyebrowRight },
        { &spec.browL, &face.eyebrowLeft },
        { &spec.mouthOuter, &face.mouthOuter },
        { &spec.mouthInner, &face.mouthInner },
        { &spec.noseFull, &face.noseFull }
    };

    //cv::Mat canvas(600, 600, CV_8UC3, cv::Scalar::all(0));

    int total = 0, start1;
    for (auto& o : objects)
    {
        if (o.first->size())
        {
            start1 = o.first->front();
            for (const auto& i : *o.first)
            {
                int index = relative ? (i - start1 + total) : i;
                if (index < points.size())
                {
                    const auto& p = points[index];
                    o.second->push_back(p);
                }
            }
            total += o.first->size();
        }
    }

    fill(face);

    return face;
}

FaceModel shapeToFace(drishti::core::Shape& shape, FaceSpecification::Format kind)
{
    auto points = shape.getPoints();

    FaceModel face;
    face.roi = shape.roi;
    face.points = points;

    // TODO: review relative
    FaceSpecification spec = FaceSpecification::create(kind);
    return shapeToFace(shape, spec, false); //(kind == FaceSpecification::kHELEN));
}

DRISHTI_FACE_NAMESPACE_END
