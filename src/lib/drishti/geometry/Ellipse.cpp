/*! -*-c++-*-
  @file   Ellipse.cpp
  @author David Hirvonen
  @brief  Implementation of an Ellipse class with various geometric operations.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/geometry/Ellipse.h"
#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_math.h"

#include <opencv2/imgproc.hpp>
#include <iostream>

DRISHTI_GEOMETRY_BEGIN

static cv::Point2f transpose(const cv::Point2f& p)
{
    return cv::Point2f(p.y, -p.x);
}
static cv::Point2f getVectorAtAngle(float angle, float length)
{
    float theta = angle * M_PI / 180.0;
    return cv::Point2f(std::cos(theta), std::sin(theta)) * length;
}

std::vector<float> pointsToPhi(const std::vector<cv::Point2f>& points)
{
    std::vector<float> phi{ points[0].x, points[1].x, points[2].x, points[3].x, points[4].x };
    return phi;
}

// Provie common ellipse control points:
cv::Point2f Ellipse::getMajorAxisPos() const
{
    return (center + getVectorAtAngle(angle, size.width / 2.0));
}
cv::Point2f Ellipse::getMajorAxisNeg() const
{
    return (center - getVectorAtAngle(angle, size.width / 2.0));
}
cv::Point2f Ellipse::getMinorAxisPos() const
{
    return (center + transpose(getVectorAtAngle(angle, size.height / 2.0)));
}
cv::Point2f Ellipse::getMinorAxisNeg() const
{
    return (center - transpose(getVectorAtAngle(angle, size.height / 2.0)));
}

// (((((((( Ellipse ))))))))
Ellipse::Ellipse(const Ellipse& src)
    : cv::RotatedRect(src)
    , m_par(src.m_par)
{
}
Ellipse::Ellipse(const cv::Vec6d& par)
    : cv::RotatedRect(conicPar2Cen(par))
    , m_par(par)
{
}
Ellipse::Ellipse(const cv::RotatedRect& cen)
    : cv::RotatedRect(cen)
    , m_par(conicCen2Par(cen))
{
}
Ellipse::Ellipse(const cv::RotatedRect& cen, const cv::Vec6d& par)
    : cv::RotatedRect(cen)
    , m_par(par)
{
}

cv::RotatedRect Ellipse::getEllipse() const
{
    return conicPar2Cen(m_par);
}

void ellipse(cv::Mat& image, const Ellipse& e, const cv::Scalar& color, int width, int type)
{
#if !DRISHTI_BUILD_MIN_SIZE
    cv::ellipse(image, e, color, width, type);
    cv::line(image, e.center, e.getMajorAxisPos(), color, width, type);
#else
    CV_Assert(false);
#endif
}

std::vector<float> ellipseToPhi(const cv::RotatedRect& e)
{
    float cx = e.center.x;
    float cy = e.center.y;
    float angle = (e.angle) * M_PI / 180.0;
    float scale = core::logN(e.size.width, 2.0f);
    float aspectRatio = core::logN(e.size.height / e.size.width, 2.0f);
    return std::vector<float>{ cx, cy, angle, scale, aspectRatio };
}

// Note: This is for ellipse drawn in transposed image
cv::RotatedRect phiToEllipse(const std::vector<float>& phi)
{
    double width = std::pow(2.0, phi[3]);
    cv::Size2f size(width, std::pow(2.0, phi[4]) * width);
    cv::RotatedRect ellipse(cv::Point2f(phi[0], phi[1]), size, phi[2] * 180.0 / M_PI);

    return ellipse;
}

// ===== from opencv/imgproc/drawing.cpp:

/*
 constructs polygon that represents elliptic arc.
 */
void ellipse2Poly(const cv::RotatedRect& ellipse, float delta, std::vector<cv::Point2f>& points)
{
    delta *= M_PI / 180.0f;
    float theta = ellipse.angle * M_PI / 180.0f;
    const float b = ellipse.size.height / 2.f;
    const float a = ellipse.size.width / 2.f;
    const float cos_theta = std::cos(theta);
    const float sin_theta = std::sin(theta);
    for (float t = 0.0; t < 2.f * M_PI; t += delta)
    {
        const float cos_t = std::cos(t);
        const float sin_t = std::sin(t);
        cv::Point2f p(a * cos_t * cos_theta - b * sin_t * sin_theta, a * cos_t * sin_theta + b * sin_t * cos_theta);
        points.push_back(ellipse.center + p);
    }
}

DRISHTI_GEOMETRY_END
