/*! -*-c++-*-
  @file   EyeModelEstimatorImpl.h
  @author David Hirvonen
  @brief  Internal declaration of eye model estimator private implementation class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_eye_EyeModelEstimatorImpl_h__
#define __drishti_eye_EyeModelEstimatorImpl_h__

#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/eye/IrisNormalizer.h"
#include "drishti/eye/EyeIO.h"
#include "drishti/ml/ShapeEstimator.h"
#include "drishti/core/Parallel.h"
#include "drishti/core/Shape.h"
#include "drishti/core/timing.h"
#include "drishti/core/Logger.h"
#include "drishti/geometry/Ellipse.h"
#include "drishti/geometry/Primitives.h"

#include <opencv2/core/core.hpp>
#include <opencv2/imgproc/imgproc.hpp>

DRISHTI_EYE_NAMESPACE_BEGIN

using EllipseVec = std::vector<cv::RotatedRect>;

#define EYE_OPENNESS_IRIS_THRESHOLD 0.10

using DRISHTI_EYE::operator*;

class EyeModelEstimator::Impl
{
public:
    using PointVec = std::vector<cv::Point2f>;

    Impl();

    Impl(const std::string& eyeRegressor, const std::string& irisRegressor = {}, const std::string& pupilRegressor = {});

    ~Impl();

    void init();

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger);

    void setEyelidStagesHint(int stages)
    {
        m_eyeEstimator->setStagesHint(stages);
    }
    int getEyelidStagesHint() const
    {
        return m_eyeEstimator->getStagesHint();
    }
    void setIrisStagesHint(int stages)
    {
        m_irisEstimator->setStagesHint(stages);
    }
    int getIrisStagesHint() const
    {
        return m_irisEstimator->getStagesHint();
    }
    void setEyelidInits(int n)
    {
        m_eyelidInits = n;
    }
    int getEyelidInits() const
    {
        return m_eyelidInits;
    }

    void setIrisInits(int n)
    {
        m_irisInits = n;
    }
    int getIrisInits() const
    {
        return m_irisInits;
    }

    void setTargetWidth(int width)
    {
        m_targetWidth = width;
    }

    void setDoPupil(bool flag)
    {
        m_doPupil = flag;
    }
    bool getDoPupil() const
    {
        return m_doPupil;
    }

    void setOpennessThreshold(float threshold)
    {
        m_opennessThrehsold = threshold;
    }
    float getOpennessThreshold() const
    {
        return m_opennessThrehsold;
    }

    // Input: grayscale for contour regression
    // Red channel is closest to NIR for iris
    // TODO: Need a lazy image conversion type
    int operator()(const cv::Mat& crop, EyeModel& eye) const;

    void normalize(const cv::Mat& crop, const EyeModel& eye, const cv::Size& size, NormalizedIris& code, int padding = 0) const
    {
        IrisNormalizer()(crop, eye, size, code, padding);
    }

    cv::Mat drawMeanShape(const cv::Size& size) const
    {
        EyeModel eye = getMeanShape(size);
        cv::Mat canvas(size, CV_8UC3, cv::Scalar::all(0));
        eye.draw(canvas);
        return canvas;
    }

    EyeModel getMeanShape(const cv::Size& size) const
    {
        EyeModel eye = shapeToEye(m_eyeEstimator->getMeanShape(), EyeModelSpecification::create(16, 9));

        auto mu = m_irisEstimator->getMeanShape();
        eye.irisEllipse = cv::RotatedRect({ mu[1].x, mu[0].x }, { mu[3].x, mu[2].x }, mu[4].x);

        const float scale = size.width;
        eye = eye * scale; // TODO: need to normalizatino mean shape during training

        eye.refine();

        return eye;
    }

    bool getDoMask() const
    {
        return m_doMask;
    }
    void setDoMask(bool flag)
    {
        m_doMask = flag;
    }

    void setDoVerbose(bool flag)
    {
        m_doVerbose = flag;
    }
    bool getDoVerbose() const
    {
        return m_doVerbose;
    }

    bool getUseHierarchy() const
    {
        return m_useHierarchy;
    }
    void setUseHierarchy(bool flag)
    {
        m_useHierarchy = flag;
    }

    bool getDoIndependentIrisAndPupil() const
    {
        return m_doIndependentIrisAndPupil;
    }
    void setDoIndependentIrisAndPupil(bool flag)
    {
        m_doIndependentIrisAndPupil = flag;
    }

    void setOptimizationLevel(int level)
    {
        m_optimizationLevel = level;
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& m_eyeEstimator;
        ar& m_irisEstimator;
        ar& m_pupilEstimator;
    }

private:
    cv::RotatedRect estimateCentralIris(const cv::Mat& I, const cv::Mat& M, const EllipseVec& irses) const;

    void segmentPupil(const cv::Mat& I, EyeModel& eye, int targetWidth = 128) const;
    void segmentIris(const cv::Mat& I, EyeModel& eye) const;
    void segmentEyelids(const cv::Mat& I, EyeModel& eye) const;
    void segmentEyelids_(const cv::Mat& I, EyeModel& eye) const; // deprecated (shape based jitter)
    std::vector<std::vector<cv::Point2f>> createInitialEyelidPoses() const;

protected:
    EyeModelSpecification m_eyeSpec;

    geometry::UniformSimilarityParams m_jitterIrisParams;
    geometry::UniformSimilarityParams m_jitterEyelidParams;

    int m_optimizationLevel = 10;
    int m_targetWidth = 256;
    bool m_doVerbose = false;
    int m_eyelidInits = 1;
    int m_irisInits = 1;

    float m_opennessThrehsold = EYE_OPENNESS_IRIS_THRESHOLD;

    bool m_doMask = false;
    bool m_useHierarchy = true;
    bool m_doPupil = true;
    bool m_doIndependentIrisAndPupil = true;

    std::unique_ptr<ml::ShapeEstimator> m_eyeEstimator;
    std::unique_ptr<ml::ShapeEstimator> m_irisEstimator;
    std::unique_ptr<ml::ShapeEstimator> m_pupilEstimator;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

template <class Archive>
void EyeModelEstimator::serialize(Archive& ar, const unsigned int version)
{
    ar& m_impl;
}

template <typename T>
T median(std::vector<T>& params)
{
    std::vector<float>::iterator nth = params.begin() + params.size() / 2;
    std::nth_element(params.begin(), nth, params.end());
    return *nth;
}

DRISHTI_EYE_NAMESPACE_END

#endif /* defined(__drishti_eye_EyeModelEstimatorImpl_h__) */
