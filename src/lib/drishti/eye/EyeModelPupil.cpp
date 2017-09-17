/*! -*-c++-*-
  @file   EyeModelPupil.cpp
  @author David Hirvonen
  @brief  Implemenation of pupile specific eye model estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of pupil specific estimation/regression used
  to create the final eye model.

*/

#include "drishti/eye/EyeModelEstimatorImpl.h"

#include "drishti/rcpr/CPR.h"

#define DEBUG_PUPIL 0

DRISHTI_EYE_NAMESPACE_BEGIN

void EyeModelEstimator::Impl::segmentPupil(const cv::Mat& I, EyeModel& eye, int targetWidth) const
{
    CV_Assert(eye.irisEllipse.size.width > 0);

    // Create a tight crop on the iris
    const float radius = std::max(eye.irisEllipse.size.width, eye.irisEllipse.size.height);
    const cv::Point2f diag(radius * 0.5f, radius * 0.5f);
    const cv::Point2f center = eye.irisEllipse.center;
    const cv::Rect roi(center - (diag * 1.0f), center + (diag * 1.0f));

    cv::Mat crop;
    cv::Point2f tl = roi.tl();
    const cv::Rect overlap = roi & cv::Rect({ 0, 0 }, I.size());

    if (overlap.area() == 0)
    {
        return;
    }

    if (overlap != roi)
    {
        crop.create(roi.size(), I.type());
        crop = cv::Scalar::all(0);
        I(overlap).copyTo(crop(overlap - roi.tl()));
    }
    else
    {
        crop = I(roi);
    }

    const float scale = float(targetWidth) / crop.cols;
    cv::resize(crop, crop, {}, scale, scale, cv::INTER_CUBIC);

    // =====================
    // Coarse iris estimate:
    cv::RotatedRect pupil((center - tl) * scale, { radius * scale / 3.f, radius * scale / 3.f }, 0.f);
    std::vector<cv::RotatedRect> pupils;
    const float minScale = radius * 1.f / 4.f;
    const float maxScale = radius * 1.f / 2.f;
    for (float s = minScale; s <= maxScale; s *= 1.05f)
    {
        pupils.emplace_back(pupil.center, cv::Size2f(s, s) * scale, pupil.angle);
    }

    std::vector<rcpr::Vector1d> params(5, rcpr::Vector1d(pupils.size()));

    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        // Find pupil:
        const auto& e = pupils[i];

        // TODO: currently override 2d point interface
        std::vector<bool> mask;
        std::vector<cv::Point2f> points = { e.center, { e.size.width, e.size.height } };
        (*m_pupilEstimator)(crop, points, mask);

        rcpr::Vector1d phi = drishti::rcpr::ellipseToPhi(geometry::pointsToEllipse(points));
        for (int j = 0; j < 5; j++)
        {
            params[j][i] = phi[j];
        }
    };

#if DEBUG_PUPIL
    m_pupilEstimator->setDoPreview(true);
    harness({ 0, int(pupils.size()) });
#else
    m_pupilEstimator->setDoPreview(false);
    cv::parallel_for_({ 0, int(pupils.size()) }, harness);
#endif

    // Find Mean
    rcpr::Vector1d model(5);
    for (int i = 0; i < 5; i++)
    {
        rcpr::Vector1d::iterator nth = params[i].begin() + params[i].size() / 2;
        std::nth_element(params[i].begin(), nth, params[i].end());
        model[i] = *nth;
    }

    eye.pupilEllipse = rcpr::phiToEllipse(model);
    eye.pupilEllipse.angle += 90.0;
    std::swap(eye.pupilEllipse.center.x, eye.pupilEllipse.center.y);

#if 0
    cv::cvtColor(crop, crop, cv::COLOR_GRAY2BGR);
    cv::ellipse(crop, pupil, {255,0,255}, 2, 8);
    cv::ellipse(crop, eye.pupilEllipse, {0,255,0}, 2, 8);
    cv::imshow("crop", crop), cv::waitKey(0);
#endif

    eye.pupilEllipse.size *= (1.0f / scale);
    eye.pupilEllipse.center *= (1.0f / scale);
    eye.pupilEllipse.center += tl;
}

DRISHTI_EYE_NAMESPACE_END
