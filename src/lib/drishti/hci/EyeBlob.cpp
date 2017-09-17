/*! -*-c++-*-
  @file   EyeBlob.cpp
  @author David Hirvonen
  @brief  Internal declaration for a utility feature point extraction.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/EyeBlob.h"
#include "drishti/geometry/motion.h"

DRISHTI_HCI_NAMESPACE_BEGIN

EyeBlobJob::EyeBlobJob(const cv::Size& size, const std::array<drishti::eye::EyeWarp, 2>& eyeWarps)
    : filtered(size)
    , alpha(size)
    , eyeWarps(eyeWarps)
{
}

void EyeBlobJob::run()
{
    cv::extractChannel(filtered, alpha, 3);
    FeaturePoints points;
    extractPoints(alpha, points, 1.f);
    for (int i = 0; i < 2; i++)
    {
        eyePoints[i] = getValidEyePoints(points, eyeWarps[i], filtered.size());
    }
}

EyeBlobJob::FeaturePoints
EyeBlobJob::getValidEyePoints(const FeaturePoints& points, const drishti::eye::EyeWarp& eyeWarp, const cv::Size& size)
{
    drishti::geometry::ConicSection_<float> C(eyeWarp.eye.irisEllipse);

    FeaturePoints pointsOnIris;
    const cv::Matx33f H = eyeWarp.H.inv() * transformation::normalize(size);
    for (const auto& f : points)
    {
        const auto& p = f.point;
        cv::Point3f q3 = H * cv::Point3f(p.x, p.y, 1.f);

        FeaturePoint q;
        q.point = { q3.x / q3.z, q3.y / q3.z };
        q.radius = f.radius;

        if (C.algebraicDistance(q.point) < 0.f)
        {
            pointsOnIris.emplace_back(q);
        }
    }

    // Additional eyelid pruning:
    const float margin = eyeWarp.eye.irisEllipse.size.width * 0.125f;
    pointsOnIris.erase(std::remove_if(pointsOnIris.begin(), pointsOnIris.end(), [&](const FeaturePoint& p) {
        const float d = cv::pointPolygonTest(eyeWarp.eye.eyelids, p.point, true);
        return d < margin;
    }),
        pointsOnIris.end());

    return pointsOnIris;
}

DRISHTI_HCI_NAMESPACE_END
