/*!
  @file   EyeModelIris.cpp
  @author David Hirvonen
  @brief  Implemenation of iris specific eye model estimation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of iris estimation/regression used
  to create the final eye model.

*/

#include "drishti/eye/EyeModelEstimatorImpl.h"
#include "drishti/core/drishti_math.h"

DRISHTI_EYE_BEGIN

#define DRISHTI_CPR_DEBUG_PHI_ESTIMATE 0

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
#  include <opencv2/highgui.hpp>
#endif

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
static void drawIrisEstimates(const cv::Mat &I, const EllipseVec &irises, const std::string &name="iris");
static cv::RotatedRect tranpose(cv::RotatedRect e);
#endif

static cv::RotatedRect ellipseFromCircle(const cv::Point2f &c, float radius, float theta);
static void jitter(cv::RNG &rng, const EyeModel &eye, const geometry::UniformSimilarityParams &params, EllipseVec &irises, int n);
static void createIrisEstimates(const EyeModel &eye, const cv::RotatedRect &pStar, EllipseVec &irises);

void EyeModelEstimator::Impl::segmentIris(const cv::Mat &I, EyeModel &eye) const
{
    DRISHTI_STREAM_LOG_FUNC(4,1,m_streamLogger);

    // Find transformation mapping mean iris to our image:
    auto cpr = dynamic_cast<drishti::rcpr::CPR*>(m_irisEstimator.get());
    CV_Assert(cpr != 0);

    cv::Mat1b M;
    if(cpr && cpr->usesMask())
    {
        M = eye.mask(I.size(), false);
    }

    // Initial iris estimates:
    EllipseVec irises;
    createIrisEstimates(eye, cpr->getPStar(), irises);

    cv::RNG rng;
    if(m_irisInits > 1)
    {
        jitter(rng, eye, m_jitterIrisParams, irises, m_irisInits-1);
    }

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    drawIrisEstimates(I, irises, "iris-in");
#endif

    if(m_useHierarchy && irises.size() > 1)
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

#if DRISHTI_CPR_TRANSPOSE
    eye.irisEllipse = tranpose(eye.irisEllipse);
#endif
}

cv::RotatedRect
EyeModelEstimator::Impl::estimateCentralIris(const cv::Mat &I, const cv::Mat &M, const EllipseVec &irises) const
{
#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    EllipseVec estimates;
#endif

    std::vector< rcpr::Vector1d > params(5, rcpr::Vector1d(irises.size()));
    std::function<void(int)> worker = [&](int i)
    {
        // Find iris:
        std::vector<bool> mask;
        std::vector<cv::Point2f> points = geometry::ellipseToPoints(irises[i]);
        ScopeTimer timer("iris_time", m_doVerbose);
        (*m_irisEstimator)(I, M, points, mask);

        rcpr::Vector1d phi = drishti::rcpr::ellipseToPhi(geometry::pointsToEllipse(points));
        for(int j = 0; j < 5; j++)
        {
            params[j][i] = phi[j];
        }

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
        estimates.push_back(drishti::rcpr::phiToEllipse(phi));
#endif
    };

    drishti::core::ParallelHomogeneousLambda harness(worker);
    m_irisEstimator->setDoPreview(true);

    // XGBoost is not reentrant:
    harness({0, int(irises.size())});
    //cv::parallel_for_({0, int(irises.size())}, harness, 2);

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
    drawIrisEstimates(I, estimates, "iris-out");
#endif

    // Find Mean
    rcpr::Vector1d model(5);
    for(int i = 0; i < 5; i++)
    {
        model[i] = geometry::median(params[i]);
    }
    return rcpr::phiToEllipse(model);
}

// ### utility functions ###

static cv::RotatedRect ellipseFromCircle(const cv::Point2f &c, float radius, float theta)
{
    return cv::RotatedRect(c, {radius*2.f,radius*2.f}, theta*180.0/M_PI);
}

// Note: The inner corner should be most stable:
cv::RotatedRect EyeModelEstimator::estimateIrisFromLimbusPoints(const EyeModel &eye)
{
    return ellipseFromCircle(*eye.irisCenter, cv::norm(*eye.irisInner - *eye.irisCenter), M_PI_2);
    //return ellipseFromCircle(eye.irisEllipse.center, cv::norm(*eye.irisInner - *eye.irisCenter), M_PI_2);
}

static void jitter(cv::RNG &rng, const EyeModel &eye, const geometry::UniformSimilarityParams &params, EllipseVec &irises, int n)
{
    // TODO: move this to a config file:
    const float icd = cv::norm(eye.getInnerCorner() - eye.getOuterCorner());
    geometry::UniformSimilarityParams params_ = params;
    params_.deltaX *= icd;
    params_.deltaY *= icd;

    EllipseVec jittered;

    for(auto &e : irises)
    {
        for(int i = 0; i < n; i++)
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

static void createIrisEstimates(const EyeModel &eye, const cv::RotatedRect &pStar, EllipseVec &irises)
{
    if(eye.irisEllipse.size.width)
    {
        irises.push_back({eye.irisEllipse.center, eye.irisEllipse.size, pStar.angle});
    }
    else
    {
        // Initial guess from point estimates:
        cv::RotatedRect guessCenter, guessInner, guessOuter;
        if(eye.irisCenter.has && eye.irisInner.has && eye.irisOuter.has)
        {
            // TODO: use interocular distance for constraint on radius;
            irises =
            {
                ellipseFromCircle(eye.irisCenter, cv::norm(*eye.irisOuter - *eye.irisInner)*0.5f, pStar.angle),
                ellipseFromCircle(eye.irisCenter, cv::norm(*eye.irisInner - *eye.irisCenter), pStar.angle),
                ellipseFromCircle(eye.irisCenter, cv::norm(*eye.irisOuter - *eye.irisCenter), pStar.angle)
            };
        }
    }
}

#if DRISHTI_CPR_DEBUG_PHI_ESTIMATE
static void drawIrisEstimates(const cv::Mat &I, const EllipseVec &irises, const std::string &name)
{
    cv::Mat canvas;
    cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);
    bool isFirst = true;
    for(const auto &e : irises)
    {
        cv::Scalar color(isFirst ? 0 : 255, 255, 0);
        geometry::ellipse(canvas, e, color, 1, 8);
        isFirst = false;
    }
    cv::imshow(name, canvas);
    cv::waitKey(0);
}

static cv::RotatedRect tranpose(cv::RotatedRect e)
{
    float theta = e.angle * M_PI / 180.0;
    std::swap(e.center.x, e.center.y);
    e.angle = std::atan2(std::cos(theta), std::sin(theta)) * 180.0 / M_PI;
    return e;
}
#endif // DRISHTI_CPR_DEBUG_PHI_ESTIMATE

DRISHTI_EYE_END
