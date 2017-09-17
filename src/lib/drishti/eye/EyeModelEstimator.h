/*! -*-c++-*-
  @file   EyeModelEstimator.h
  @author David Hirvonen
  @brief  Internal eye model estimator declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the internal SDK eye model estimator,
  which does the actual work associated with generating eye models.

*/

#ifndef __drishti_eye_EyeModelEstimator_h__
#define __drishti_eye_EyeModelEstimator_h__

#include "drishti/core/drishti_defs.hpp"
#include "drishti/eye/drishti_eye.h"
#include "drishti/eye/Eye.h"
#include "drishti/eye/NormalizedIris.h"
#include "drishti/core/Logger.h"

#include <memory>

DRISHTI_EYE_NAMESPACE_BEGIN

class EyeModelEstimator
{
public:
    struct RegressorConfig
    {
        std::string eyeRegressor;
        std::string irisRegressor;
        std::string pupilRegressor;
    };

    EyeModelEstimator();
    EyeModelEstimator(const std::string& filename);
    EyeModelEstimator(std::istream& is, const std::string& hint = {});
    EyeModelEstimator(const RegressorConfig& config);
    ~EyeModelEstimator();

    bool good() const;
    operator bool() const;

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger);

    virtual int operator()(const cv::Mat& crop, EyeModel& eye) const;

    void setOpennessThreshold(float threshold);
    float getOpennessThreshold() const;

    static cv::RotatedRect estimateIrisFromLimbusPoints(const EyeModel& eye);

    void normalize(const cv::Mat& crop, const EyeModel& eye, const cv::Size& size, NormalizedIris& code, int padding = 0) const;

    void setDoIndependentIrisAndPupil(bool flag);

    void setTargetWidth(int width);
    void setDoPupil(bool flag);
    bool getDoPupil() const;

    void setEyelidStagesHint(int stages);
    int getEyelidStagesHint() const;

    void setIrisStagesHint(int stages);
    int getIrisStagesHint() const;

    void setEyelidInits(int n);
    int getEyelidInits() const;

    void setIrisInits(int n);
    int getIrisInits() const;

    cv::Mat drawMeanShape(const cv::Size& size) const;

    bool getDoMask() const;
    void setDoMask(bool flag);

    bool getDoVerbose() const;
    void setDoVerbose(bool flag);

    bool getUseHierarchy() const;
    void setUseHierarchy(bool flag);

    void setOptimizationLevel(int level);

    EyeModel getMeanShape(const cv::Size& size) const;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    class Impl; // keep public for implementatino file serialization versioning

protected:
    std::shared_ptr<spdlog::logger> m_streamLogger;

    std::unique_ptr<Impl> m_impl;
};

DRISHTI_EYE_NAMESPACE_END

#endif /* defined(__drishti_eye_EyeModelEstimator_h__) */
