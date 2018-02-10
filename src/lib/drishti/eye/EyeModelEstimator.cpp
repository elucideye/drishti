
/*!
  @file   EyeModelEstimator.cpp
  @author David Hirvonen
  @brief  Internal eye model estimator implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the internal SDK eye model estimator,
  which does the actual work associated with generating eye models.

*/

#include "drishti/eye/EyeModelEstimatorImpl.h"

#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/make_unique.h"
#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/rcpr/CPR.h"

#include <fstream>

#define DRISHTI_EYE_USE_DARK_CHANNEL 0

DRISHTI_EYE_NAMESPACE_BEGIN

#if DRISHTI_EYE_USE_DARK_CHANNEL
static cv::Mat getDarkChannel(const cv::Mat& I);
#endif
static float resizeEye(const cv::Mat& src, cv::Mat& dst, float width);

EyeModelEstimator::Impl::Impl()
{
    init();
}

EyeModelEstimator::Impl::Impl(const std::string& eyeRegressor, const std::string& irisRegressor, const std::string& pupilRegressor)
{
    m_eyeEstimator = drishti::core::make_unique<drishti::ml::RegressionTreeEnsembleShapeEstimator>(eyeRegressor);
    if (!irisRegressor.empty())
    {
        m_irisEstimator = make_unique_cpb<drishti::rcpr::CPR>(irisRegressor);
        if (m_irisEstimator && !pupilRegressor.empty())
        {
            m_pupilEstimator = make_unique_cpb<drishti::rcpr::CPR>(pupilRegressor);
        }
    }

    init();
}

EyeModelEstimator::Impl::~Impl() = default;

void EyeModelEstimator::Impl::init()
{
    // Jitter iris defaults (normalized by eyelid extents):
    m_jitterIrisParams.theta = { static_cast<float>(-M_PI / 64.f), static_cast<float>(+M_PI / 64.f) };
    m_jitterIrisParams.scale = { 7.f / 8.f, 8.f / 7.f };
    m_jitterIrisParams.deltaX = { -0.125f, +0.125f };
    m_jitterIrisParams.deltaY = { -0.050f, +0.050f };

    // Jitter eyelid defaults (normalized by image extents):
    m_jitterEyelidParams.theta = { static_cast<float>(-M_PI / 32.f), static_cast<float>(+M_PI / 32.f) };
    m_jitterEyelidParams.scale = { 3.0f / 4.0f, 1.0f }; //
    m_jitterEyelidParams.deltaX = { -0.05f, +0.05f };
    m_jitterEyelidParams.deltaY = { -0.05f, +0.05f };

    m_eyeSpec = EyeModelSpecification::create(16, 9, 1, 1, 1, 1, 1);
}

void EyeModelEstimator::Impl::setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
{
    m_streamLogger = logger;
    if (m_irisEstimator)
    {
        m_irisEstimator->setStreamLogger(logger);
        m_eyeEstimator->setStreamLogger(logger);
    }
}

// Input: grayscale for contour regression
// Red channel is closest to NIR for iris
// TODO: Need a lazy image conversion type

int EyeModelEstimator::Impl::operator()(const cv::Mat& crop, EyeModel& eye) const
{
    cv::Mat I;
    float scale = resizeEye(crop, I, m_targetWidth), scaleInv = (1.0 / scale);

    cv::Mat Ic[3]{ I }, dark, blue, red;
    if (I.channels() == 3)
    {
        cv::split(I, Ic);

#if DRISHTI_EYE_USE_DARK_CHANNEL
        // Dark channel:
        dark = getDarkChannel(I);
#endif
        blue = Ic[0];
        red = Ic[2];
    }
    else
    {
        //dark = I;
        blue = red = I;
    }

    // ######## Find the eyelids #########
    segmentEyelids(blue, eye);

    if (m_doIndependentIrisAndPupil)
    {
        float openness = 0.f;
        if ((openness = eye.openness()) > m_opennessThrehsold)
        {
            // ((((( Do iris estimate )))))
            if (m_irisEstimator)
            {
                segmentIris(red, eye);

                {
                    // If point-wise estimates match the iris regressor, then update our landmarks
                    cv::Point2f irisCenter, innerLimbus, outerLimbus;
                    eye.estimateIrisLandmarks(irisCenter, innerLimbus, outerLimbus);
                    eye.irisCenter = irisCenter;
                    eye.irisInner = innerLimbus;
                    eye.irisOuter = outerLimbus;
                }

                eye.iris = 0.f; // drop the circular initial estimate
                eye.pupil = 0.f;
                eye.pupilEllipse.center = eye.irisEllipse.center;

                if (m_pupilEstimator && m_doPupil && eye.irisEllipse.size.area() > 0.f)
                {
                    segmentPupil(red, eye);
                }
            }
        }
        else
        {
            // for squinting eyes defer to limbus point estimate:
            eye.irisEllipse = estimateIrisFromLimbusPoints(eye);
            eye.pupilEllipse.center = eye.irisEllipse.center;
        }
    }

    // Scale up the model
    if (scaleInv != 1.0f)
    {
        eye = eye * scaleInv;
    }

    return 0;
}

EyeModelEstimator::EyeModelEstimator()
{
}

EyeModelEstimator::EyeModelEstimator(std::istream& is, const std::string& hint)
{
    load_cpb(is, *this);
}

EyeModelEstimator::EyeModelEstimator(const std::string& filename)
{
    load_cpb(filename, *this);
}

EyeModelEstimator::EyeModelEstimator(const RegressorConfig& config)
{
    m_impl = drishti::core::make_unique<EyeModelEstimator::Impl>(config.eyeRegressor, config.irisRegressor, config.pupilRegressor);
}

EyeModelEstimator::~EyeModelEstimator() {}

void EyeModelEstimator::setDoIndependentIrisAndPupil(bool flag)
{
    m_impl->setDoIndependentIrisAndPupil(flag);
}

bool EyeModelEstimator::good() const
{
    return static_cast<bool>(m_impl.get());
}

EyeModelEstimator::operator bool() const
{
    return good();
}

void EyeModelEstimator::setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
{
    m_streamLogger = logger;
    if (m_impl)
    {
        m_impl->setStreamLogger(m_streamLogger);
    }
}

int EyeModelEstimator::operator()(const cv::Mat& crop, EyeModel& eye) const
{
    return (*m_impl)(crop, eye);
}

void EyeModelEstimator::normalize(const cv::Mat& crop, const EyeModel& eye, const cv::Size& size, NormalizedIris& code, int padding) const
{
    return m_impl->normalize(crop, eye, size, code, padding);
}

void EyeModelEstimator::setEyelidInits(int n)
{
    m_impl->setEyelidInits(n);
}

int EyeModelEstimator::getEyelidInits() const
{
    return m_impl->getEyelidInits();
}

void EyeModelEstimator::setIrisInits(int n)
{
    m_impl->setIrisInits(n);
}

int EyeModelEstimator::getIrisInits() const
{
    return m_impl->getIrisInits();
}

void EyeModelEstimator::setOptimizationLevel(int level)
{
    m_impl->setOptimizationLevel(level);
}

void EyeModelEstimator::setTargetWidth(int width)
{
    m_impl->setTargetWidth(width);
}

void EyeModelEstimator::setOpennessThreshold(float threshold)
{
    m_impl->setOpennessThreshold(threshold);
}
float EyeModelEstimator::getOpennessThreshold() const
{
    return m_impl->getOpennessThreshold();
}

void EyeModelEstimator::setDoPupil(bool flag)
{
    m_impl->setDoPupil(flag);
}
bool EyeModelEstimator::getDoPupil() const
{
    return m_impl->getDoPupil();
}

void EyeModelEstimator::setDoVerbose(bool flag)
{
    m_impl->setDoVerbose(flag);
}
bool EyeModelEstimator::getDoVerbose() const
{
    return m_impl->getDoVerbose();
}

cv::Mat EyeModelEstimator::drawMeanShape(const cv::Size& size) const
{
    return m_impl->drawMeanShape(size);
}

DRISHTI_EYE::EyeModel EyeModelEstimator::getMeanShape(const cv::Size& size) const
{
    return m_impl->getMeanShape(size);
}

bool EyeModelEstimator::getDoMask() const
{
    return m_impl->getDoMask();
}

void EyeModelEstimator::setDoMask(bool flag)
{
    m_impl->setDoMask(flag);
}

bool EyeModelEstimator::getUseHierarchy() const
{
    return m_impl->getUseHierarchy();
}
void EyeModelEstimator::setUseHierarchy(bool flag)
{
    m_impl->setUseHierarchy(flag);
}
void EyeModelEstimator::setEyelidStagesHint(int stages)
{
    m_impl->setEyelidStagesHint(stages);
}
int EyeModelEstimator::getEyelidStagesHint() const
{
    return m_impl->getEyelidStagesHint();
}

void EyeModelEstimator::setIrisStagesHint(int stages)
{
    m_impl->setIrisStagesHint(stages);
}
int EyeModelEstimator::getIrisStagesHint() const
{
    return m_impl->getIrisStagesHint();
}

static float resizeEye(const cv::Mat& src, cv::Mat& dst, float width)
{
    float scale = 1.f;
    if (src.cols < width)
    {
        dst = src;
    }
    else
    {
        scale = float(width) / float(src.cols);
        cv::resize(src, dst, {}, scale, scale, cv::INTER_CUBIC);
    }
    return scale;
}

#if DRISHTI_EYE_USE_DARK_CHANNEL
static cv::Mat getDarkChannel(const cv::Mat& I)
{
    cv::Mat dark;
    cv::Mat Ic = I.isContinuous() ? I : I.clone();
    cv::reduce(Ic.reshape(1, I.size().area()), dark, 1, CV_REDUCE_MIN);
    dark = dark.reshape(1, I.rows);
    return dark;
}
#endif

DRISHTI_EYE_NAMESPACE_END
