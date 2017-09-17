/*! -*-c++-*-
  @file   Primitives.cpp
  @author David Hirvonen
  @brief  Declaration of geometric routines and primitives.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.

*/

#include "drishti/geometry/Primitives.h"

DRISHTI_GEOMETRY_BEGIN

cv::Point2f mean(const PointVec& points)
{
    cv::Point2f mu;
    for (auto& p : points)
    {
        mu += p;
    }
    return (mu * (1.f / float(points.size())));
}

cv::Matx33f procrustes(const PointVec& points)
{
    // Estimate translation:
    cv::Point2f mu = mean(points);
    cv::Matx33f T(1, 0, -mu.x, 0, 1, -mu.y, 0, 0, 1);

    // Estimate scale:
    float scale = 0.f;
    for (const auto& p : points)
    {
        scale += ((p - mu).dot(p - mu));
    }

    scale = std::sqrt(scale / float(points.size()));
    cv::Matx33f S(cv::Matx33f::diag({ 1.f / scale, 1.f / scale, 1.f }));

    cv::Matx33f H = S * T;

    return H;
}

// WRT ellipse center
cv::RotatedRect randomSimilarityEllipse(const UniformSimilarityParams& params, cv::RNG& rng)
{
    const float theta = rng.uniform(params.theta[0], params.theta[1]) * M_PI / 180.0;
    const float s1 = rng.uniform(params.scale[0], params.scale[1]);
    const float x = rng.uniform(params.deltaX[0], params.deltaX[1]);
    const float y = rng.uniform(params.deltaY[0], params.deltaY[1]);
    return cv::RotatedRect({ x, y }, { s1, s1 }, theta);
}

// WRT specified center
cv::Matx33f randomSimilarity(const UniformSimilarityParams& params, cv::RNG& rng, const cv::Point2f& center, bool rotation)
{
    cv::Matx33f C1(1, 0, -center.x, 0, 1, -center.y, 0, 0, 1);
    cv::Matx33f C2(1, 0, +center.x, 0, 1, +center.y, 0, 0, 1);

    const float s1 = rng.uniform(params.scale[0], params.scale[1]);
    const cv::Matx33f S1(cv::Matx33f::diag({ s1, s1, 1 }));

    const float x = rng.uniform(params.deltaX[0], params.deltaX[1]);
    const float y = rng.uniform(params.deltaY[0], params.deltaY[1]);
    const cv::Matx33f T(1, 0, x, 0, 1, y, 0, 0, 1);

    cv::Matx33f R = cv::Matx33f::eye();
    if (rotation)
    {
        const float theta = rng.uniform(params.theta[0], params.theta[1]) * M_PI / 180.0;
        const float ct = std::cos(theta);
        const float st = std::sin(theta);
        R = cv::Matx33f(+ct, -st, 0, +st, +ct, 0, 0, 0, 1);
    }

    cv::Matx33f H = C2 * T * S1 * R * C1;

    return H;
}

DRISHTI_GEOMETRY_END
