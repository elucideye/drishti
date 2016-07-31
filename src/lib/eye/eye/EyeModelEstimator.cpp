/*!
  @file   EyeModelEstimator.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Internal eye model estimator implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the implementation of the internal SDK eye model estimator,
  which does the actual work associated with generating eye models.

*/

#include "eye/EyeModelEstimatorImpl.h"
#include "core/boost_serialize_common.h"

DRISHTI_EYE_BEGIN

static float resizeEye(const cv::Mat &src, cv::Mat &dst, float width)
{
    float scale = 1.f;
    if(src.cols < width)
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

static cv::Mat getDarkChannel(const cv::Mat &I)
{
    cv::Mat dark;
    cv::Mat Ic = I.isContinuous() ? I : I.clone();
    cv::reduce(Ic.reshape(1, I.size().area()), dark, 1, CV_REDUCE_MIN);
    dark = dark.reshape(1, I.rows);
    return dark;
}


// TODO: support for stream input
EyeModelEstimator::Impl::Impl(const std::string &eyeRegressor, const std::string &irisRegressor, const std::string &pupilRegressor)
{
    DRISHTI_STREAM_LOG_FUNC(2,1,m_streamLogger);

    m_eyeEstimator = std::make_shared<ml::RegressionTreeEnsembleShapeEstimator>(eyeRegressor);
    if(!irisRegressor.empty())
    {
        std::shared_ptr< drishti::rcpr::CPR > irisEstimator = std::make_shared<drishti::rcpr::CPR>();
        load_pba_z(irisRegressor, *irisEstimator);
        m_irisEstimator = irisEstimator;

        if(m_irisEstimator && !pupilRegressor.empty())
        {
            std::shared_ptr< drishti::rcpr::CPR > pupilEstimator = std::make_shared<drishti::rcpr::CPR>();
            load_pba_z(pupilRegressor, pupilEstimator);
            m_pupilEstimator = pupilEstimator;
        }
    }

    init();
}

EyeModelEstimator::Impl::Impl()
{
    DRISHTI_STREAM_LOG_FUNC(2,2,m_streamLogger);

    init();
}

void EyeModelEstimator::Impl::init()
{
    DRISHTI_STREAM_LOG_FUNC(2,3,m_streamLogger);

    // Jitter iris defaults (normalized by eyelid extents):
    m_jitterIrisParams.theta = { -M_PI/64.f, +M_PI/64.f };
    m_jitterIrisParams.scale = { 7.f/8.f, 8.f/7.f };
    m_jitterIrisParams.deltaX = { -0.125, +0.125 };
    m_jitterIrisParams.deltaY = { -0.05, +0.05 };

    // Jitter eyelid defaults (normalized by image extents):
    m_jitterEyelidParams.theta = { -M_PI/32.f, +M_PI/32.f };
    m_jitterEyelidParams.scale = { 3.0/4.0, 1.0 }; //
    m_jitterEyelidParams.deltaX = { -0.05, +0.05 };
    m_jitterEyelidParams.deltaY = { -0.05, +0.05 };

    m_eyeSpec = EyeModelSpecification::create(16,9,1,1,1,1,1);
}

void EyeModelEstimator::Impl::setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
{
    m_streamLogger = logger;
    if(m_irisEstimator)
    {
        m_irisEstimator->setStreamLogger(logger);
        m_eyeEstimator->setStreamLogger(logger);
    }
}

// Input: grayscale for contour regression
// Red channel is closest to NIR for iris
// TODO: Need a lazy image conversion type

int EyeModelEstimator::Impl::operator()(const cv::Mat &crop, EyeModel &eye) const
{
    DRISHTI_STREAM_LOG_FUNC(2,4,m_streamLogger);

    cv::Mat I;
    float scale = resizeEye(crop, I, m_targetWidth), scaleInv = (1.0 / scale);

    cv::Mat Ic[3] { I }, dark, blue, red;
    if(I.channels() == 3)
    {
        cv::split(I, Ic);

        // Dark channel:
        //dark = getDarkChannel(I);
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

    if(m_doIndependentIrisAndPupil)
    {
        float openness = 0.f;
        if((openness=eye.openness()) > m_opennessThrehsold)
        {
            // ((((( Do iris estimate )))))
            if(m_irisEstimator)
            {
                segmentIris(red, eye);

                // TODO:
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

                //cv::Mat canvas;
                //cv::cvtColor(dark, canvas, cv::COLOR_GRAY2BGR);
                //cv::ellipse(canvas, eye.irisEllipse, {0,255,0}, 1, 8); cv::imshow("debug", canvas); cv::waitKey(0);
                if(m_pupilEstimator && m_doPupil && eye.irisEllipse.size.area() > 0.f)
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
    if(scaleInv != 1.0f)
    {
        eye = eye * scaleInv;
    }

    return 0;
}

//====

// Public API:
// TODO: support for stream input
EyeModelEstimator::EyeModelEstimator(const RegressorConfig &config)
{
    DRISHTI_STREAM_LOG_FUNC(2,5,m_streamLogger);
    m_impl = std::make_shared<EyeModelEstimator::Impl>(config.eyeRegressor, config.irisRegressor, config.pupilRegressor);
}

EyeModelEstimator::EyeModelEstimator(const std::string &eyeRegressor, const std::string &irisRegressor, const std::string &pupilRegressor)
{
    DRISHTI_STREAM_LOG_FUNC(2,6,m_streamLogger);
    m_impl = std::make_shared<EyeModelEstimator::Impl>(eyeRegressor, irisRegressor, pupilRegressor);
}

EyeModelEstimator::~EyeModelEstimator() {}

void EyeModelEstimator::setDoIndependentIrisAndPupil(bool flag)
{
    DRISHTI_STREAM_LOG_FUNC(2,7,m_streamLogger);
    m_impl->setDoIndependentIrisAndPupil(flag);
}

void EyeModelEstimator::setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
{
    DRISHTI_STREAM_LOG_FUNC(2,8,m_streamLogger);
    m_streamLogger = logger;
    if(m_impl)
    {
        m_impl->setStreamLogger(m_streamLogger);
    }
}

int EyeModelEstimator::operator()(const cv::Mat &crop, EyeModel &eye) const
{
    DRISHTI_STREAM_LOG_FUNC(2,9,m_streamLogger);
    return (*m_impl)(crop, eye);
}

void EyeModelEstimator::normalize(const cv::Mat &crop, const EyeModel &eye, const cv::Size &size, NormalizedIris &code, int padding) const
{
    DRISHTI_STREAM_LOG_FUNC(2,10,m_streamLogger);
    return m_impl->normalize(crop, eye, size, code, padding);
}

void EyeModelEstimator::normalize(const cv::Mat &crop, const std::vector<cv::Point2f> &curve, const cv::Size &size, NormalizedCurve &code) const
{
    DRISHTI_STREAM_LOG_FUNC(2,11,m_streamLogger);
    return m_impl->normalize(crop, curve, size, code);
}

void EyeModelEstimator::setEyelidInits(int n)
{
    DRISHTI_STREAM_LOG_FUNC(2,12,m_streamLogger);
    m_impl->setEyelidInits(n);
}

int EyeModelEstimator::getEyelidInits() const
{
    DRISHTI_STREAM_LOG_FUNC(2,13,m_streamLogger);
    return m_impl->getEyelidInits();
}

void EyeModelEstimator::setIrisInits(int n)
{
    DRISHTI_STREAM_LOG_FUNC(2,14,m_streamLogger);
    m_impl->setIrisInits(n);
}

int EyeModelEstimator::getIrisInits() const
{
    DRISHTI_STREAM_LOG_FUNC(2,15,m_streamLogger);
    return m_impl->getIrisInits();
}

void EyeModelEstimator::setOptimizationLevel(int level)
{
    DRISHTI_STREAM_LOG_FUNC(2,16,m_streamLogger);
    m_impl->setOptimizationLevel(level);
}

void EyeModelEstimator::setTargetWidth(int width)
{
    DRISHTI_STREAM_LOG_FUNC(2,17,m_streamLogger);
    m_impl->setTargetWidth(width);
}

void EyeModelEstimator::setOpennessThreshold(float threshold)
{
    DRISHTI_STREAM_LOG_FUNC(2,18,m_streamLogger);
    m_impl->setOpennessThreshold(threshold);
}
float EyeModelEstimator::getOpennessThreshold() const
{
    DRISHTI_STREAM_LOG_FUNC(2,19,m_streamLogger);
    return m_impl->getOpennessThreshold();
}

void EyeModelEstimator::setDoPupil(bool flag)
{
    DRISHTI_STREAM_LOG_FUNC(2,20,m_streamLogger);
    m_impl->setDoPupil(flag);
}
bool EyeModelEstimator::getDoPupil() const
{
    DRISHTI_STREAM_LOG_FUNC(2,21,m_streamLogger);
    return m_impl->getDoPupil();
}

void EyeModelEstimator::setDoVerbose(bool flag)
{
    DRISHTI_STREAM_LOG_FUNC(2,22,m_streamLogger);
    m_impl->setDoVerbose(flag);
}
bool EyeModelEstimator::getDoVerbose() const
{
    DRISHTI_STREAM_LOG_FUNC(2,23,m_streamLogger);
    return m_impl->getDoVerbose();
}

cv::Mat EyeModelEstimator::drawMeanShape(const cv::Size  &size) const
{
    DRISHTI_STREAM_LOG_FUNC(2,24,m_streamLogger);
    return m_impl->drawMeanShape(size);
}

DRISHTI_EYE::EyeModel EyeModelEstimator::getMeanShape(const cv::Size &size) const
{
    DRISHTI_STREAM_LOG_FUNC(2,25,m_streamLogger);
    return m_impl->getMeanShape(size);
}

int EyeModelEstimator::load(std::istream &is, EyeModelEstimator &eme)
{
    load_pba_z(is, eme);
    return 0;
}

bool EyeModelEstimator::getDoMask() const
{
    DRISHTI_STREAM_LOG_FUNC(2,26,m_streamLogger);
    return m_impl->getDoMask();
}

void EyeModelEstimator::setDoMask(bool flag)
{
    DRISHTI_STREAM_LOG_FUNC(2,27,m_streamLogger);
    m_impl->setDoMask(flag);
}

bool EyeModelEstimator::getUseHierarchy() const
{
    DRISHTI_STREAM_LOG_FUNC(2,28,m_streamLogger);
    return m_impl->getUseHierarchy();
}
void EyeModelEstimator::setUseHierarchy(bool flag)
{
    DRISHTI_STREAM_LOG_FUNC(2,29,m_streamLogger);
    m_impl->setUseHierarchy(flag);
}

void EyeModelEstimator::setEyelidStagesHint(int stages)
{
    DRISHTI_STREAM_LOG_FUNC(2,30,m_streamLogger);
    m_impl->setEyelidStagesHint(stages);
}
int EyeModelEstimator::getEyelidStagesHint() const
{
    DRISHTI_STREAM_LOG_FUNC(2,31,m_streamLogger);
    return m_impl->getEyelidStagesHint();
}

void EyeModelEstimator::setIrisStagesHint(int stages)
{
    DRISHTI_STREAM_LOG_FUNC(2,32,m_streamLogger);
    m_impl->setIrisStagesHint(stages);
}
int EyeModelEstimator::getIrisStagesHint() const
{
    DRISHTI_STREAM_LOG_FUNC(2,33,m_streamLogger);
    return m_impl->getIrisStagesHint();
}

void EyeModelEstimator::setIrisStagesRepetitionFactor(int x)
{
    DRISHTI_STREAM_LOG_FUNC(2,34,m_streamLogger);
    m_impl->setIrisStagesRepetitionFactor(x);
}
int EyeModelEstimator::getIrisStagesRepetitionFactor() const
{
    DRISHTI_STREAM_LOG_FUNC(2,35,m_streamLogger);
    return m_impl->getIrisStagesRepetitionFactor();
}

int EyeModelEstimator::load(const std::string &filename, EyeModelEstimator &eme)
{
    std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
    return load(ifs, eme);
}

// Boost serialization:
template<class Archive> void EyeModelEstimator::serialize(Archive & ar, const unsigned int version)
{
    assert(version >= 1);
    ar & m_impl;
}

//template void EyeModelEstimator::Impl::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
//template void EyeModelEstimator::Impl::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
//template void EyeModelEstimator::Impl::serialize<boost::archive::binary_oarchive>(boost::archive::binary_oarchive &ar, const unsigned int);
//template void EyeModelEstimator::Impl::serialize<boost::archive::binary_iarchive>(boost::archive::binary_iarchive &ar, const unsigned int);
template void EyeModelEstimator::Impl::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void EyeModelEstimator::Impl::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

//template void EyeModelEstimator::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
//template void EyeModelEstimator::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
//template void EyeModelEstimator::serialize<boost::archive::binary_oarchive>(boost::archive::binary_oarchive &ar, const unsigned int);
//template void EyeModelEstimator::serialize<boost::archive::binary_iarchive>(boost::archive::binary_iarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

DRISHTI_EYE_END

BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator::Impl);
