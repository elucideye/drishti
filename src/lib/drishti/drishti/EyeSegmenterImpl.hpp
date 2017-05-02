/**
  @file   EyeSegmenterImpl.hpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains the declaration of the private implementation structure
  used for the top level SDK -- this provides a cleaner API, minimizes dependencies
  and improve build times.
*/

#ifndef __drishti_drishti_EyeSegmenterImpl_hpp__
#define __drishti_drishti_EyeSegmenterImpl_hpp__

#include "drishti/EyeSegmenter.hpp"
#include "drishti/core/Logger.h"

#include "drishti/eye/Eye.h"
#include "drishti/drishti_cv.hpp"

// clang-format off
namespace drishti { namespace eye { class EyeModelEstimator; } };
// clang-format on

_DRISHTI_SDK_BEGIN

class DRISHTI_EXPORT EyeSegmenter::Impl
{
public:
    Impl(bool doLoad = false);
    Impl(const std::string& filename, ArchiveKind kind);
    Impl(std::istream& is, ArchiveKind kind);
    ~Impl();
    int operator()(const Image3b& image, Eye& eye, bool isRight);

    Eye getMeanEye(int width) const;

    void setEyelidInits(int count);
    int getEyelidInits() const;

    void setIrisInits(int count);
    int getIrisInits() const;

    static int getMinWidth();

    static float getRequiredAspectRatio();

    void setOptimizationLevel(int level);

protected:
    void init(std::istream& is, ArchiveKind);

    std::unique_ptr<eye::EyeModelEstimator> m_eme;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

// Maintain lightweight inline conversions in private header for internal use
inline drishti::sdk::Eye convert(const drishti::eye::EyeModel& model)
{
    drishti::sdk::Eye e;

    const auto& inner = model.getInnerCorner();
    const auto& outer = model.getOuterCorner();

    e.setIris(cvToDrishti(model.irisEllipse));
    e.setPupil(cvToDrishti(model.pupilEllipse));
    e.setCorners(cvToDrishti(inner), cvToDrishti(outer));
    e.setRoi(cvToDrishti(model.roi.has ? *model.roi : cv::Rect()));
    e.setEyelids(drishti::sdk::cvToDrishti(model.eyelidsSpline));
    e.setCrease(drishti::sdk::cvToDrishti(model.creaseSpline));

    return e;
}

inline drishti::eye::EyeModel convert(const drishti::sdk::Eye& eye)
{
    drishti::eye::EyeModel e;

    const auto& inner = eye.getInnerCorner();
    const auto& outer = eye.getOuterCorner();

    e.irisEllipse = drishtiToCv(eye.getIris());
    e.pupilEllipse = drishtiToCv(eye.getPupil());
    e.innerCorner = drishtiToCv(inner);
    e.outerCorner = drishtiToCv(outer);
    e.eyelidsSpline = drishtiToCv(eye.getEyelids());
    e.creaseSpline = drishtiToCv(eye.getCrease());

    return e;
}

_DRISHTI_SDK_END

#endif /* defined(__drishti_drishti_EyeSegmenterImpl_hpp__) */
