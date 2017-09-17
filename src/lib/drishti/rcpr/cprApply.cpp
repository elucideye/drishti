/*! -*-c++-*-
  @file   cprApply.cpp
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Implementation of core CPR ellipse model prediction.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/rcpr/drishti_rcpr.h"
#include "drishti/rcpr/CPR.h"

#include "drishti/core/timing.h"

#include "drishti/geometry/Ellipse.h"

#define DRISHTI_CPR_DO_DEBUG 0

// clang-format off
#if DRISHTI_CPR_DO_DEBUG
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif
// clang-format on

DRISHTI_RCPR_NAMESPACE_BEGIN

void CPR::CPROpts::merge(const CPR::CPROpts& opts, int checkExtra)
{
    pInit.merge(opts.pInit, checkExtra);
    K.merge(opts.K, checkExtra);
    rad.merge(opts.rad, checkExtra);
}

std::ostream& operator<<(std::ostream& os, const cv::RotatedRect& e)
{
    os << e.center << "," << e.size << " " << e.angle;
    return os;
}

int CPR::cprApplyTree(const cv::Mat& I, const RegModel& regModel, const Vector1d& pIn, CPRResult& result, bool doPreview) const
{
    cv::Mat1b mask = cv::Mat1b::ones(I.size());
    return cprApplyTree(ImageMaskPair(I, mask), regModel, pIn, result, doPreview);
}

int CPR::cprApplyTree(const ImageMaskPair& IsIn, const RegModel& regModel, const Vector1d& pIn, CPRResult& result, bool doPreview) const
{
    ImageMaskPair Is = IsIn;
    auto& p = result.p;
    p = pIn;

    // Apply each single stage regressor, starting from pose p:
    auto& model = *(regModel.model);
    auto T = *(regModel.T);
    result.pAll.resize(T); // store result at end of each stage

    // repeat the whole thing 2x
    std::vector<int> stage;

    // Create a recipe for executing the stages:
    for (int i = 0; i < std::min(stagesHint, int(T)); i++)
    {
        stage.push_back(i);
    }

    for (const auto& t : stage)
    {
        auto& reg = *(*(regModel.regs))[t];

        FeaturesResult ftrResult;
        featuresComp(model, p, Is, *(reg.ftrData), ftrResult);

        auto pDel = identity(model);

        cv::Mat features;
        cv::Mat(ftrResult.ftrs).reshape(1, 1).copyTo(features);
        features.convertTo(features, CV_32F);

        {
            // XGBOOST
            std::vector<std::vector<float>> data(features.rows);
            for (int i = 0; i < features.rows; i++)
            {
                data[i] = features.row(i);
            }

            for (auto& t : reg.xgbdt)
            {
                pDel[t.first] = (*t.second)(data[0]);
            }
        }

        p = compose(model, p, pDel);
        result.pAll[t] = p; // store result for this stage

#if DRISHTI_CPR_DO_DEBUG && !HAS_XGBOOST
        // TODO: Legacy non xgboost
        if (doPreview)
        {
            cv::Mat canvas;
            cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);

            const auto e = phiToEllipse(p), eIn = phiToEllipse(pIn);
            cv::ellipse(canvas, eIn, { 255, 0, 0 }, 1, 8);
            cv::ellipse(canvas, e, { 0, 255, 0 }, 1, 8);

            drishti::geometry::Ellipse e2(e);
            cv::line(canvas, e.center, e2.getMajorAxisPos(), { 0, 255, 0 }, 1, 8);
            cv::imshow("I", canvas); // opt
            cv::waitKey(0);
        }
#endif
    }
    return 0;
}

DRISHTI_RCPR_NAMESPACE_END
