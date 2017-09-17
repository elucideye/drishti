/*! -*-c++-*-
  @file   Shape.h
  @author David Hirvonen
  @brief  Declaration of simple (de)serializable shapes/contours.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_Shape_h__
#define __drishti_core_Shape_h__

#include "drishti/core/drishti_core.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

#include <iostream>
#include <iomanip>
#include <stdio.h>

DRISHTI_CORE_NAMESPACE_BEGIN

using PointVec = std::vector<cv::Point2f>;

void fitSpline(const PointVec& controlPoints, PointVec& interpolatedPoints, int count = 64, bool closed = true);

void upsample(const PointVec& controlPoints, PointVec& interpolatedPoints, int factor, bool closed);

template <typename T>
inline cv::Point_<T> centroid(const std::vector<cv::Point_<T>>& contour, int n = std::numeric_limits<int>::max())
{
    cv::Point_<T> c(0, 0);
    n = std::min(int(contour.size()), n);
    for (int i = 0; i < n; i++)
    {
        c += contour[i];
    }
    return c / T(n);
}

struct ControlPoint
{
    ControlPoint() {}
    ControlPoint(float x, float y, bool knot)
        : p(x, y)
        , knot(knot)
    {
    }
    ControlPoint(const cv::Point2f& position, bool k = false)
        : p(position)
        , knot(k)
    {
    }
    ControlPoint& operator=(const cv::Point& p)
    {
        this->p = p;
        this->knot = false;
        return (*this);
    }
    ControlPoint& operator=(const cv::Point2f& p)
    {
        this->p = p;
        this->knot = false;
        return (*this);
    }
    operator cv::Point() const
    {
        return p;
    }
    cv::Point2f p;
    bool knot;
};

struct Shape
{
    typedef std::vector<ControlPoint> ControlPointSetType;
    typedef ControlPointSetType::iterator IteratorType;
    IteratorType begin()
    {
        return contour.begin();
    }
    IteratorType end()
    {
        return contour.end();
    }

    Shape() {}
    Shape(const cv::Rect& bbox, double score_ = 0.0)
        : roi(bbox)
        , score(score_)
    {
    }
    Shape(const cv::Rect& bbox, const std::vector<cv::Point2f>& points, double score_ = 0.0)
        : roi(bbox)
        , score(score_)
    {
        for (const auto& p : points)
        {
            contour.emplace_back(ControlPoint(p, false));
        }
    }

    Shape& operator=(const cv::Rect& bbox)
    {
        roi = bbox;
        contour = {};
        return *this;
    }

    cv::Point2f centroid() const
    {
        return drishti::core::centroid(getPoints());
    }

    std::vector<cv::Point2f> getPoints() const
    {
        std::vector<cv::Point2f> points;
        for (const auto& p : contour)
        {
            points.push_back(p.p);
        }

        return points;
    }

    void clear()
    {
        contour.clear();
        roi = {};
    }

    void write(cv::FileStorage& fs) const;
    void read(const cv::FileNode& node);

    void write(const std::string& filename) const;
    void read(const std::string& filename);

    // (((( Data members ))))

    ControlPointSetType contour;
    cv::Rect roi;
    double score = 0.0;
};

template <typename T>
Shape operator*(const cv::Matx<T, 3, 3>& H, const Shape& src)
{
    auto dst = src;
    for (auto& p : dst.contour)
    {
        cv::Point3_<T> q = H * cv::Point3_<T>(p.p.x, p.p.y, 1.f);
        p.p = { q.x / q.z, q.y / q.z };
    }
    return dst;
}

struct AnnotatedContourImage
{
    // Add your output here as desired
    std::string filename;
    std::vector<Shape> contours;
};

// These functions must be within the same top level drishti namespace (opencv limitation)
void write(cv::FileStorage& fs, const std::string&, const drishti::core::Shape& x);
void read(const cv::FileNode& node, drishti::core::Shape& x, const drishti::core::Shape& default_value = {});

// TODO: move this somewhere else
std::vector<cv::Vec3b> makeRainbow();

DRISHTI_CORE_NAMESPACE_END

#endif /* defined(__drishti_core_Shape_h__) */
