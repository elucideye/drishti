/*! -*-c++-*-
  @file   Cylinder.h
  @author David Hirvonen
  @brief  Abstraction for cylinder object.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.

*/

#ifndef __drishti_geometry_Cylinder_h__
#define __drishti_geometry_Cylinder_h__

#include "drishti/geometry/drishti_geometry.h"
#include "drishti/geometry/Mesh3D.h"

#include <opencv2/core.hpp>

DRISHTI_GEOMETRY_BEGIN

template <typename T>
class Cylinder3D
{
public:
    Cylinder3D() {}

    Cylinder3D(const Cylinder3D& src)
        : position(src.position)
        , radius(src.radius)
        , length(src.length)
    {
    }

    Cylinder3D(const cv::Point3_<T>& position, T radius, T length)
        : position(position)
        , radius(radius)
        , length(length)
    {
    }

    const cv::Point3_<T>& getPosition() const { return position; }
    T getRadius() const { return radius; }
    T getLength() const { return length; }

protected:
    cv::Point3_<T> position;
    T radius = T(0);
    T length = T(0);
};

template <typename T>
Mesh3D<T> drawCylinder(T length, T radius, int n, T arrow = T(0.1))
{
    arrow = std::min(std::max(arrow, T(0.0)), T(1.0));
    const float start = length * (T(1.0) - arrow);

    Mesh3D<T> points;
    for (int i = 0; i < n; i++)
    {
        const float theta = (2.f * M_PI) * static_cast<float>(i) / static_cast<float>(n);
        const float x1 = std::cos(theta) * radius, x2 = x1 * 2.f;
        const float y1 = std::sin(theta) * radius, y2 = y1 * 2.f;
        points.push_back(cv::Point3_<T>(x1, y1, T(0.0)));
        points.push_back(cv::Point3_<T>(x1, y1, start));

        points.push_back(cv::Point3_<T>(x2, y2, start));
        points.push_back(cv::Point3_<T>(T(0.0), T(0.0), length));
    }

    return points;
}

template <typename T>
std::array<Mesh3D<T>, 3> drawAxes(const cv::Point3_<T>& axes, T radius, int n, T arrow)
{
    auto zAxis = drawCylinder(axes.z, radius, n, arrow);
    auto xAxis = drawCylinder(axes.x, radius, n, arrow);
    auto yAxis = drawCylinder(axes.y, radius, n, arrow);
    for (auto& p : xAxis)
    {
        p = { +p.z, +p.x, +p.y };
    }
    for (auto& p : yAxis)
    {
        p = { +p.x, +p.z, +p.y };
    }
    for (auto& p : zAxis)
    {
        p = { +p.x, +p.y, -p.z };
    }
    return std::array<Mesh3D<T>, 3>{ { xAxis, yAxis, zAxis } };
}

DRISHTI_GEOMETRY_END

#endif // __drishti_geometry_Cylinder_h__
