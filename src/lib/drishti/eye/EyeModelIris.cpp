/*! -*-c++-*-
  @file   EyeModelIris.cpp
  @author David Hirvonen
  @brief  Implemenation of iris specific eye model estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of iris estimation/regression used
  to create the final eye model.

*/

#include "drishti/eye/EyeModelEstimatorImpl.h"
#include "drishti/rcpr/CPR.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST

DRISHTI_EYE_NAMESPACE_BEGIN

#define DRISHTI_CPR_DEBUG_PHI_ESTIMATE 0

// clang-format off
#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
#  include <opencv2/highgui.hpp>
#endif
// clang-format on

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
static void drawIrisEstimates(const cv::Mat& I, const EllipseVec& irises, const std::string& name = "iris");
#endif

static void jitter(cv::RNG& rng, const EyeModel& eye, const geometry::UniformSimilarityParams& params, EllipseVec& irises, int n);

void EyeModelEstimator::Impl::segmentIris(const cv::Mat& I, EyeModel& eye) const
{
    // Find transformation mapping mean iris to our image:
    auto cpr = dynamic_cast<drishti::rcpr::CPR*>(m_irisEstimator.get());
    CV_Assert(cpr != 0);

    cv::Mat1b M;
    if (cpr && cpr->usesMask())
    {
        M = eye.mask(I.size(), false);
    }

    // Initial iris estimates:
    EllipseVec irises{ { eye.irisEllipse.center, eye.irisEllipse.size, cpr->getPStar().angle } };

    cv::RNG rng;
    if (m_irisInits > 1)
    {
        jitter(rng, eye, m_jitterIrisParams, irises, m_irisInits - 1);
    }

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    drawIrisEstimates(I, irises, "iris-in");
#endif

    if (m_useHierarchy && irises.size() > 1)
    {
        eye.irisEllipse = estimateCentralIris(I, M, irises);
    }
    else
    {
        std::vector<bool> mask;
        std::vector<cv::Point2f> points = geometry::ellipseToPoints(irises[0]);
        (*m_irisEstimator)(I, M, points, mask);
        eye.iris = 0;
        eye.irisEllipse = geometry::pointsToEllipse(points);
    }
}

cv::RotatedRect
EyeModelEstimator::Impl::estimateCentralIris(const cv::Mat& I, const cv::Mat& M, const EllipseVec& irises) const
{
#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    EllipseVec estimates;
#endif

    std::vector<rcpr::Vector1d> params(5, rcpr::Vector1d(irises.size()));

    drishti::core::ParallelHomogeneousLambda harness = [&](int i) {
        // Find iris:
        std::vector<bool> mask;
        std::vector<cv::Point2f> points = geometry::ellipseToPoints(irises[i]);
        (*m_irisEstimator)(I, M, points, mask);

        rcpr::Vector1d phi = drishti::rcpr::ellipseToPhi(geometry::pointsToEllipse(points));
        for (int j = 0; j < 5; j++)
        {
            params[j][i] = phi[j];
        }

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
        estimates.push_back(drishti::rcpr::phiToEllipse(phi));
#endif
    };

    m_irisEstimator->setDoPreview(true);

    // XGBoost is not reentrant:
    harness({ 0, int(irises.size()) });
//cv::parallel_for_({0, int(irises.size())}, harness, 2);

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    drawIrisEstimates(I, estimates, "iris-out");
#endif

    // Find Mean
    rcpr::Vector1d model(5);
    for (int i = 0; i < 5; i++)
    {
        model[i] = geometry::median(params[i]);
    }
    return rcpr::phiToEllipse(model);
}

// ### utility functions ###

static cv::RotatedRect ellipseFromCircle(const cv::Point2f& c, float radius, float theta)
{
    return cv::RotatedRect(c, { radius * 2.f, radius * 2.f }, theta * 180.0 / M_PI);
}

// Note: The inner corner should be most stable:
cv::RotatedRect EyeModelEstimator::estimateIrisFromLimbusPoints(const EyeModel& eye)
{
    return ellipseFromCircle(*eye.irisCenter, cv::norm(*eye.irisInner - *eye.irisCenter), M_PI_2);
    //return ellipseFromCircle(eye.irisEllipse.center, cv::norm(*eye.irisInner - *eye.irisCenter), M_PI_2);
}

static void jitter(cv::RNG& rng, const EyeModel& eye, const geometry::UniformSimilarityParams& params, EllipseVec& irises, int n)
{
    // TODO: move this to a config file:
    const float icd = cv::norm(eye.getInnerCorner() - eye.getOuterCorner());
    geometry::UniformSimilarityParams params_ = params;
    params_.deltaX *= icd;
    params_.deltaY *= icd;

    EllipseVec jittered;

    for (auto& e : irises)
    {
        for (int i = 0; i < n; i++)
        {
            //const cv::Matx33f H = geometry::randomSimilarity(params_, rng, eye.irisEllipse.center);
            //cv::RotatedRect iris = H * e;

            // Here we apply the transformation in the native ellipse representation (instead of generic conic):
            cv::RotatedRect iris = geometry::randomSimilarityEllipse(params_, rng);
            iris.size.width *= e.size.width;
            iris.size.height *= e.size.width;
            iris.center += e.center;
            iris.angle += e.angle;

            DRISHTI_EYE::EyeModel::normalizeEllipse(iris);
            jittered.push_back(iris);
        }
    }

    std::copy(jittered.begin(), jittered.end(), std::back_inserter(irises));
}

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
static void drawIrisEstimates(const cv::Mat& I, const EllipseVec& irises, const std::string& name)
{
    cv::Mat canvas;
    cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);
    bool isFirst = true;
    for (const auto& e : irises)
    {
        cv::Scalar color(isFirst ? 0 : 255, 255, 0);
        geometry::ellipse(canvas, e, color, 1, 8);
        isFirst = false;
    }
    cv::imshow(name, canvas);
    cv::waitKey(0);
}
#endif // DRISHTI_CPR_DEBUG_PHI_ESTIMATE

DRISHTI_EYE_NAMESPACE_END
