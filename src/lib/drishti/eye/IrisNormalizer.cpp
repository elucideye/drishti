/*! -*-c++-*-
  @file   IrisNormalizer.cpp
  @author David Hirvonen
  @brief  Implementation of internal class for creating ellipso-polar normalized iris images.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/IrisNormalizer.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/geometry/intersectConicLine.h"

#include <opencv2/imgproc.hpp>

#include <iostream>

DRISHTI_EYE_NAMESPACE_BEGIN

IrisNormalizer::IrisNormalizer()
{
}

void IrisNormalizer::warpIris(const cv::Mat& crop, const cv::Mat1b& mask, const cv::Size& paddedSize, Rays& rayPixels, Rays& rayTexels, NormalizedIris& code, int padding) const
{
    code.getRoi() = cv::Rect({ padding, 0 }, paddedSize - cv::Size(2 * padding, 0));
    code.getPaddedMask().create(paddedSize, CV_8UC1);
    code.getPaddedImage().create(paddedSize, crop.type());
    code.getPaddedImage() = cv::Scalar::all(0);

    cv::Mat1f mapX(paddedSize), mapY(paddedSize);

    for (int x = 0; x < paddedSize.width; x++)
    {
        for (int y = 0; y < paddedSize.height; y++)
        {
            const float alpha = (y + 1) / float(paddedSize.height), beta = (1.0 - alpha);
            const auto& pi = rayPixels[x][0];
            const auto& pp = rayPixels[x][1];
            const cv::Point2f u = (pi * alpha) + (pp * beta);
            mapX(y, x + padding) = u.x;
            mapY(y, x + padding) = u.y;
        }
    }

    cv::remap(crop, code.getPaddedImage(), mapX, mapY, cv::INTER_CUBIC);
    cv::remap(mask, code.getPaddedMask(), mapX, mapY, cv::INTER_NEAREST);
}

cv::Size IrisNormalizer::createRays(const EyeModel& eye, const cv::Size& size, Rays& rayPixels, Rays& rayTexels, int padding) const
{
    cv::Matx33f iris = drishti::geometry::ConicSection_<float>(eye.irisEllipse).getMatrix();
    cv::Matx33f pupil = drishti::geometry::ConicSection_<float>(eye.pupilEllipse).getMatrix();
    cv::Size paddedSize = size + cv::Size(2 * padding, 0);

    rayPixels.reserve(paddedSize.width);
    rayTexels.reserve(paddedSize.width);

    /* int n = 0; */
    cv::Vec3f P[2];
    cv::Point2f p[2], pi, pp;
    for (int x = -padding; x < (size.width + padding); x++)
    {
        const float theta = float((x + size.width) % size.width) / size.width * float(2.0 * M_PI);
        cv::Point3f v(std::cos(theta), std::sin(theta), 0);
        cv::Point3f c(eye.pupilEllipse.center.x, eye.pupilEllipse.center.y, 1.f);
        const cv::Point3f L = c.cross(c + v);

        // TODO: should be able to avoid need for the dot product
        /* n = */ drishti::geometry::intersectConicLine(iris, L, P);
        p[0] = { P[0][0] / P[0][2], P[0][1] / P[0][2] };
        p[1] = { P[1][0] / P[1][2], P[1][1] / P[1][2] };
        pi = p[cv::Point2f(v.x, v.y).dot(p[0] - cv::Point2f(c.x, c.y)) > 0];

        /* n = */ drishti::geometry::intersectConicLine(pupil, L, P);
        p[0] = { P[0][0] / P[0][2], P[0][1] / P[0][2] };
        p[1] = { P[1][0] / P[1][2], P[1][1] / P[1][2] };
        pp = p[cv::Point2f(v.x, v.y).dot(p[0] - cv::Point2f(c.x, c.y)) > 0];
        Ray rayPixel = { { pp, pi } };

        // Add corresponding ray in normalized coordinates:
        cv::Point2f tp(float(x + padding) / paddedSize.width, 0.0);
        cv::Point2f ti(tp.x, 1.0);
        Ray rayTexel = { { tp, ti } };

        rayPixels.emplace_back(rayPixel);
        rayTexels.emplace_back(rayTexel);
    }

    return paddedSize;
}

void IrisNormalizer::operator()(const cv::Mat& crop, const EyeModel& eye, const cv::Size& size, NormalizedIris& code, int padding) const
{
    cv::Mat mask = eye.irisMask(crop.size());

    Rays rayPixels, rayTexels;
    cv::Size paddedSize = createRays(eye, size, rayPixels, rayTexels, padding);
    warpIris(crop, mask, paddedSize, rayPixels, rayTexels, code, padding);
}

DRISHTI_EYE_NAMESPACE_END
