/*! -*-c++-*-
  @file   CPR.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @author P. Dollár (original matlab code (random ferns))
  @brief  Declaration of Cascaded Pose Regression class (single ellipse model).

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/rcpr/CPR.h"

#include <acf/ACFField.h>

// Deprecated backwards compatibility

// clang-format off
#if !DRISHTI_CPR_DO_LEAN
#  include "cvmatio/MatlabIO.hpp"
#  include "cvmatio/MatlabIOContainer.hpp"
#  include "drishti/acf/ACFIO.h"
#endif
// clang-format on

DRISHTI_RCPR_NAMESPACE_BEGIN

CPR::CPR() {}
CPR::CPR(const CPR& src) {}

#if !DRISHTI_CPR_DO_LEAN
CPR::CPR(const std::string& filename)
{
    deserialize(filename);
}

CPR::CPR(const char* filename)
{
    deserialize(filename);
}
#endif

CPR::~CPR() = default;

bool CPR::usesMask() const
{
    bool flag = false;
    for (const auto& r : cprPrm->cascadeRecipes)
    {
        if (r.doMask)
        {
            flag = true;
            break;
        }
    }
    return flag;
}

void CPR::setDoPreview(bool flag)
{
    m_doPreview = flag;
}

std::vector<cv::Point2f> CPR::getMeanShape() const
{
    Vector1d mu = regModel->pStar;
    cv::RotatedRect e = phiToEllipse(mu);
    return std::vector<cv::Point2f>{ { e.center.x, 0 }, { e.center.y, 0 }, { e.size.width, 0 }, { e.size.height }, { e.angle, 0 } };
}

cv::RotatedRect CPR::getPStar() const
{
    Vector1d mu = regModel->pStar_;
    return phiToEllipse(mu);
}

void CPR::CprPrm::FtrPrm::merge(const CPR::CprPrm::FtrPrm& opts, int checkExtra)
{
    type.merge(opts.type, checkExtra);
    F.merge(opts.F, checkExtra);
    radius.merge(opts.radius, checkExtra);
    nChn.merge(opts.nChn, checkExtra);
}

// TODO: rethink shape estimator API to better support parametric models
// For now we'll just store each parameter in the x coordinate of a 2D point

static cv::RotatedRect pointsToEllipse(const std::vector<cv::Point2f>& points)
{
    return cv::RotatedRect({ points[0].x, points[1].x }, { points[2].x, points[3].x }, points[4].x);
}

static Vector1d pointsToPhi(const std::vector<cv::Point2f>& points)
{
    return ellipseToPhi(pointsToEllipse(points));
}

int CPR::operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const
{
    CPRResult result;

    if (m_isMat)
    {
#if DRISHTI_CPR_DO_LEAN
        std::cerr << "Software configured without random fern regressors: see DRISHTI_CPR_DO_LEAN" << std::endl;
        CV_Assert(m_isMat == false);
#else
        CPROpts opts;
        if (points.size() == 1)
        {
            opts.pInit = { "pInit", { RealType(points[0].x), RealType(points[0].y) } };
        }

        cprApply(I, (*regModel), result, opts);
#endif
    }
    else
    {
        ImageMaskPair Is{ I, M };
        Vector1d pStar = (points.size() == 5) ? pointsToPhi(points) : (*regModel->pStar);
        cprApplyTree(Is, *regModel, pStar, result, m_doPreview);
    }

    if (result.p.size() == 5)
    {
        points.resize(5);
        cv::RotatedRect ellipse = phiToEllipse(result.p);
        points = {
            { ellipse.center.x, 0.f }, // tranpose center
            { ellipse.center.y, 0.f },
            { ellipse.size.width, 0.f }, // flip width and height
            { ellipse.size.height, 0.f },
            { ellipse.angle, 0.f }
        };
    }

    return 0;
}

int CPR::operator()(const cv::Mat& I, std::vector<cv::Point2f>& points, std::vector<bool>& mask) const
{
    return (*this)(I, {}, points, mask);
}

DRISHTI_RCPR_NAMESPACE_END
