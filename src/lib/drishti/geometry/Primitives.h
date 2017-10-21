/*! -*-c++-*-
  @file   Primitives.h
  @author David Hirvonen
  @brief  Declaration of geometric routines and primitives.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_Primitives_h__
#define __drishti_geometry_Primitives_h__

#include "drishti/geometry/drishti_geometry.h"

#include <opencv2/core/core.hpp>

#include <array>

DRISHTI_GEOMETRY_BEGIN

typedef std::vector<cv::Point2f> PointVec;

// Translation and scale:
cv::Matx33f procrustes(const PointVec& points);

template <typename T>
T median(std::vector<T>& params)
{
    typename std::vector<T>::iterator nth = params.begin() + params.size() / 2;
    std::nth_element(params.begin(), nth, params.end());
    return *nth;
}

template <typename T>
std::vector<cv::Point_<T>> median(std::vector<std::vector<T>>& x, std::vector<std::vector<T>>& y)
{
    CV_Assert(x.size() == y.size());
    std::vector<cv::Point2f> result(x.size());
    for (int i = 0; i < x.size(); i++)
    {
        result[i].x = median(x[i]);
        result[i].y = median(y[i]);
    }

    return result;
}

template <typename T>
std::vector<cv::Point_<T>> pointMedian(const std::vector<std::vector<cv::Point_<T>>>& points)
{
    std::vector<std::vector<T>> x(points[0].size());
    std::vector<std::vector<T>> y(points[0].size());
    for (int i = 0; i < points.size(); i++)
    {
        for (int j = 0; j < points[i].size(); j++)
        {
            x[j].push_back(points[i][j].x);
            y[j].push_back(points[i][j].y);
        }
    }

    return median(x, y);
}

template <typename T>
cv::Point_<T> pointMedian(const std::vector<cv::Point_<T>>& points)
{
    std::vector<T> x(points.size());
    std::vector<T> y(points.size());
    for (int i = 0; i < points.size(); i++)
    {
        x[i] = points[i].x;
        y[i] = points[i].y;
    }
    return cv::Point_<T>(median(x), median(y));
}

cv::Point2f mean(const PointVec& points);

struct Point4f
{
    cv::Point2f position;
    cv::Point2f orientation;
    cv::Point2f phase;
};

inline cv::Vec4f line(const cv::Point2f& p, const cv::Point2f& q)
{
    return cv::Vec4f(p.x, p.y, p.x - q.x, p.y - q.y);
}

inline cv::Point2f project2(const cv::Vec4f& line, const cv::Point2f& q)
{
    const cv::Point2f p(line[0], line[1]), v(line[2], line[3]), d = q - p;
    return p + (d.dot(v) / v.dot(v)) * v;
}

// TODO: move this to a transformation class:
struct UniformSimilarityParams
{
    cv::Vec2f scale = { 1.0f, 1.5f };
    cv::Vec2f deltaX = { -0.33f, +0.33f };
    cv::Vec2f deltaY = { -0.50f, +0.50f };
    cv::Vec2f theta = { -10.0f * static_cast<float>(M_PI) / 180.0f, 10.0f * static_cast<float>(M_PI) / 180.0f };
};

cv::RotatedRect randomSimilarityEllipse(const UniformSimilarityParams& params, cv::RNG& rng);
cv::Matx33f randomSimilarity(const UniformSimilarityParams& params, cv::RNG& rng, const cv::Point2f& center = { 0.f, 0.f }, bool rotation = true);

// Rotate cv::Point
template <typename T>
cv::Point_<T> transpose(const cv::Point_<T>& src)
{
    return cv::Point_<T>(src.y, src.x);
}

template <typename T>
cv::Point_<T> flip(const cv::Point_<T>& src, const cv::Size_<T>& size)
{
    return cv::Point_<T>(src.x, size.height - src.y);
}

template <typename T>
cv::Point_<T> flop(const cv::Point_<T>& src, const cv::Size_<T>& size)
{
    return cv::Point_<T>(size.width - src.x, src.y);
}

// Rotate cv::Rect
template <typename T>
cv::Rect_<T> transpose(const cv::Rect_<T>& src)
{
    return cv::Rect_<T>(src.y, src.x, src.height, src.width);
}

template <typename T>
cv::Rect_<T> flip(const cv::Rect_<T>& src, const cv::Size_<T>& size)
{
    return cv::Rect_<T>({ src.x, size.height - (src.y + src.height) }, src.size());
}

template <typename T>
cv::Rect_<T> flop(const cv::Rect_<T>& src, const cv::Size_<T>& size)
{
    return cv::Rect_<T>({ size.width - (src.x + src.width) }, src.y, src.size());
}

DRISHTI_GEOMETRY_END

#endif
