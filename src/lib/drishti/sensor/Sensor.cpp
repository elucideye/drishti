/*! -*-c++-*-
  @file   Sensor.cpp
  @author David Hirvonen
  @brief  Implementation of a class to abstract a sensor/camera.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/sensor/Sensor.h"

DRISHTI_SENSOR_NAMESPACE_BEGIN

SensorModel::Intrinsic::Intrinsic()
{
}

SensorModel::Intrinsic::Intrinsic(const cv::Point2f& c, float fx, const cv::Size& size)
    : m_size(size)
    , m_c(c)
    , m_fx(fx)
{
}

cv::Matx33f SensorModel::Intrinsic::getK() const
{
    return cv::Matx33f(*m_fx, 0, m_c->x, 0, *m_fx, m_c->y, 0, 0, 1);
}

cv::Point3f SensorModel::Intrinsic::getDepth(const std::array<cv::Point2f, 2>& pixels, float widthMeters) const
{
    // Estimate the 3D position using simple IPD constant:
    const cv::Point2f p = (pixels[0] + pixels[1]) * 0.5f;
    const float widthPixels = cv::norm(pixels[0] - pixels[1]);
    const float Z = widthMeters * m_fx / widthPixels;
    return cv::Point3f(-(p.x - m_c->x) * Z / *m_fx, -(p.y - m_c->y) * Z / *m_fx, Z);
}

DRISHTI_SENSOR_NAMESPACE_END
