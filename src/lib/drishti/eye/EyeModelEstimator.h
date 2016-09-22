/*!
  @file   EyeModelEstimator.h
  @author David Hirvonen
  @brief  Internal eye model estimator declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the internal SDK eye model estimator,
  which does the actual work associated with generating eye models.

*/

#ifndef __drishtisdk__EyeModelEstimator__
#define __drishtisdk__EyeModelEstimator__

#include <stdio.h>

#include "drishti/core/drishti_defs.hpp"
#include "drishti/eye/drishti_eye.h"
#include "drishti/eye/Eye.h"
#include "drishti/eye/NormalizedIris.h"
#include "drishti/core/serialization.h"
#include "drishti/core/Logger.h"

#include <memory>

DRISHTI_EYE_BEGIN

class EyeModelEstimator
{
public:
    class Impl;

    struct RegressorConfig
    {
        std::string eyeRegressor;
        std::string irisRegressor;
        std::string pupilRegressor;
    };

    // TODO: stream input
    EyeModelEstimator() {}
    EyeModelEstimator(const RegressorConfig &config);
    EyeModelEstimator(const std::string &eye, const std::string &iris= {}, const std::string &pupil=std::string());
    virtual ~EyeModelEstimator();

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger);

    virtual int operator()(const cv::Mat &crop, EyeModel &eye) const;

    void setOpennessThreshold(float threshold);
    float getOpennessThreshold() const;

    static cv::RotatedRect estimateIrisFromLimbusPoints(const EyeModel &eye);

    void normalize(const cv::Mat &crop, const EyeModel &eye, const cv::Size &size, NormalizedIris &code, int padding=0) const;
    void normalize(const cv::Mat &crop, const std::vector<cv::Point2f> &curve, const cv::Size &size, NormalizedCurve &code) const;

    void setDoIndependentIrisAndPupil(bool flag);

    void setTargetWidth(int width);
    void setDoPupil(bool flag);
    bool getDoPupil() const;

    void setEyelidStagesHint(int stages);
    int getEyelidStagesHint() const;

    void setIrisStagesHint(int stages);
    int getIrisStagesHint() const;

    void setIrisStagesRepetitionFactor(int x);
    int getIrisStagesRepetitionFactor() const;

    void setEyelidInits(int n);
    int getEyelidInits() const;

    void setIrisInits(int n);
    int getIrisInits() const;

    cv::Mat drawMeanShape(const cv::Size &size) const;

    bool getDoMask() const;
    void setDoMask(bool flag);

    bool getDoVerbose() const;
    void setDoVerbose(bool flag);

    bool getUseHierarchy() const;
    void setUseHierarchy(bool flag);

    void setOptimizationLevel(int level);

    static int loadPBA(const std::string &filename, EyeModelEstimator &eme);
    static int loadPBA(std::istream &is, EyeModelEstimator &eme);

    static int loadTXT(const std::string &filename, EyeModelEstimator &eme);
    static int loadTXT(std::istream &is, EyeModelEstimator &eme);

    EyeModel getMeanShape(const cv::Size &size) const;

protected:

    std::shared_ptr<spdlog::logger> m_streamLogger;

    // Boost serialization:
    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version);

    std::shared_ptr<Impl> m_impl;
};

DRISHTI_EYE_END

BOOST_CLASS_EXPORT_KEY(DRISHTI_EYE::EyeModelEstimator);
BOOST_CLASS_EXPORT_KEY(DRISHTI_EYE::EyeModelEstimator::Impl);

BOOST_CLASS_VERSION(DRISHTI_EYE::EyeModelEstimator, 1);
BOOST_CLASS_VERSION(DRISHTI_EYE::EyeModelEstimator::Impl, 1);

#endif /* defined(__drishtisdk__EyeModelEstimator__) */
