/*!
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

#if DRISHTI_CPR_DO_DEBUG
#  include <opencv2/imgproc/imgproc.hpp>
#  include <opencv2/highgui/highgui.hpp>
#endif

DRISHTI_RCPR_BEGIN

#if !DRISHTI_CPR_DO_LEAN

// Apply multi stage pose regressor.
//
// USAGE
//  [p,pAll,Vs] = cprApply( Is, regModel, [varargin] )
//
// INPUTS
//  Is       - [w x h x nChn x N] input images
//  regModel - learned multi stage pose regressor
//  varargin - additional params (struct or name/value pairs)
//   .pInit    - [MxR] or [Mx2] initial pose (defaults to pStar)
//   .imgIds   - [Mx1] image id for each pose (optional if M==N)
//   .K        - [1] number of initial pose restarts
//   .rad      - [1] radius of Gassian Parzen window for finding mode
//   .chunk    - [inf] run pose clustering in chunks (to save memory)
//
// OUTPUTS
//  p        - [MxR] pose returned by multi stage regressor
//  pAll     - [MxRxT+1] pose returned at every stage of regression
//  Vs       - [wxh]x{T} visualization of ftr locations (if N==1)
//
// EXAMPLE
//
// See also cprTrain
//
// Cascaded Pose Regression Toolbox      Version 1.00
// Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
// Please email me if you find bugs, or have suggestions or questions!
// Licensed under the Simplified BSD License [see bsd.txt]

int CPR::cprApply( const cv::Mat &I, const RegModel &regModel, CPRResult &result, const CPROpts & pOpts ) const
{
    DRISHTI_STREAM_LOG_FUNC(9,1,m_streamLogger);

    CPROpts opts = pOpts;
    {
        // Provide defaults if missing:
        CPROpts dfs;
        dfs.K = { "K", 1 };
        dfs.rad = { "rad", 1 };
        opts.merge(dfs, 1);
    }

    auto p = *(opts.pInit); // copy only
    auto K = *(opts.K);
    auto rad = *(opts.rad);

    if(p.empty())
    {
        std::copy((*regModel.pStar).begin(), (*regModel.pStar).begin() + 2, std::back_inserter(p));
    }

    // Run regressor starting from a single pose:
    if(K == 1)
    {
        ScopeTimer timer("cpr");
        if(p.size() == 2) // just center, augment model params
        {
            std::copy((*regModel.pStar).begin()+2,  (*regModel.pStar).end(), std::back_inserter(p));
        }
        cprApply1(I, regModel, p, result);
    }

    //cv::Mat canvas;
    //cv::cvtColor(I.t(), canvas, cv::COLOR_GRAY2BGR);
    //cv::ellipse(canvas, phiToEllipse(result.p), {0,255,0}, 1, 8);
    //cv::imshow("canvas", canvas.t()); // opt
    //cv::waitKey(0);
    
    return 0;
}

int CPR::cprApply1( const cv::Mat &I, const RegModel &regModel, const Vector1d &pIn, CPRResult &result )
{
    cv::Mat It = I.t();

    auto &p = result.p;
    p = pIn;

    // Apply each single stage regressor, starting from pose p:
    auto &model = *(regModel.model);
    auto T = *(regModel.T);
    result.pAll.resize(T); // store result at end of each stage

    for(int t = 0; t < T; t++)
    {
        auto &reg = *(*(regModel.regs))[t];
        uint32_t r = *(reg.r);

        FeaturesResult ftrResult;
        featuresComp( model, p, It, *(reg.ftrData), ftrResult );

        FernResult fernResult;
        fernsRegApply( ftrResult.ftrs, *(reg.ferns), fernResult, {} );

        const auto &del = fernResult.ys;
        auto pDel = identity(model);
        pDel[r-1] = del; // BASE ZERO

        p = compose(model, p, pDel);
    }

    return 0;
}


//function inds = fernsInds( data, fids, thrs )
// Compute indices for each input by each fern.
//
//  USAGE
//   inds = fernsInds( data, fids, thrs )
//
//  INPUTS
//   data     - [NxF] N length F binary feature vectors
//   fids     - [MxS] feature ids for each fern for each depth
//   thrs     - [MxS] threshold corresponding to each fid
//
//  OUTPUTS
//   inds     - [NxM] computed indices for each input by each fern
//
//  EXAMPLE
//
//  See also fernsClfTrain, fernsClfApply
//
//  Piotr's Image&Video Toolbox      Version 2.50
//  Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
//  Please email me if you find bugs, or have suggestions or questions!
//  Licensed under the Simplified BSD License [see external/bsd.txt]

static uint32_t fernsIndx(const Vector1d &data, const std::vector<uint32_t> &fids, const Vector1d &thrs)
{
    size_t index = 0;
    for(size_t i = 0; i < fids.size(); i++)
    {
        index *= 2;
        if(data[fids[i]-1] < thrs[i])
        {
            index ++;
        }
    }
    return index; // matlab increments for base 1 indexing
}

//// function [ys,ysCum] = fernsRegApply( data, ferns, inds )
//  Apply learned fern regressor.
//
//  USAGE
//   [ys,ysCum] = fernsRegApply( data, ferns, [inds] )
//
//  INPUTS
//   data     - [NxF] N length F binary feature vectors
//   ferns    - learned fern regression model
//   inds     - [NxM] cached inds (from previous call to fernsInds)
//
//  OUTPUTS
//   ys       - [Nx1] predicted output values
//   ysCum    - [NxM] predicted output values after each regressor
//
//  EXAMPLE
//
//  See also fernsRegTrain, fernsInds
//
//  Piotr's Image&Video Toolbox      Version 2.50
//  Copyright 2012 Piotr Dollar.  [pdollar-at-caltech.edu]
//  Please email me if you find bugs, or have suggestions or questions!
//  Licensed under the Simplified BSD License [see external/bsd.txt]

int CPR::fernsRegApply(const Vector1d &data, const RegModel::Regs::Ferns &ferns, FernResult &result, const std::vector<uint32_t> &indsIn)
{
    size_t index = fernsIndx( data, *(ferns.fids), *(ferns.thrs) );
    result.ys = (*ferns.ysFern)[index];
    return 0;
}

#endif // !DRISHTI_CPR_DO_LEAN

void CPR::CPROpts::merge(const CPR::CPROpts &opts, int checkExtra)
{
    pInit.merge(opts.pInit, checkExtra);
    K.merge(opts.K, checkExtra);
    rad.merge(opts.rad, checkExtra);
}

std::ostream & operator<<(std::ostream &os, const cv::RotatedRect &e)
{
    os << e.center << ","  << e.size << " " << e.angle;
    return os;
}

int CPR::cprApplyTree(const cv::Mat &I, const RegModel &regModel, const Vector1d &pIn, CPRResult &result, bool doPreview) const
{
    return cprApplyTree(I, regModel, pIn, result, doPreview);
}

#define STAGE_REPETITION_FACTOR 1

int CPR::cprApplyTree(const ImageMaskPair &IsIn, const RegModel &regModel, const Vector1d &pIn, CPRResult &result, bool doPreview) const
{
    DRISHTI_STREAM_LOG_FUNC(9,2,m_streamLogger);

    ImageMaskPair Is = IsIn;
    if(DRISHTI_CPR_TRANSPOSE)
    {
        Is.getImage() = Is.getImage().t();
        Is.getMask() = Is.getMask().t();
    }

    auto &p = result.p;
    p = pIn;

    // Apply each single stage regressor, starting from pose p:
    auto &model = *(regModel.model);
    auto T = *(regModel.T);
    result.pAll.resize(T); // store result at end of each stage

    // repeat the whole thing 2x
    std::vector<int> stage;

    // Create a recipe for executing the stages:
    for(int i = 0; i < std::min(stagesHint, int(T)); i++)
    {
        for(int j = 0; j < std::max(1, stagesRepetitionFactor); j++)
        {
            stage.push_back(i);
        }
    }

    //for(int t = 0; t < T; t++)
    for(const auto &t : stage)
    {
        auto &reg = *(*(regModel.regs))[t];

        DRISHTI_STREAM_LOG_FUNC(9,3,m_streamLogger);
        FeaturesResult ftrResult;
        featuresComp( model, p, Is, *(reg.ftrData), ftrResult);
        DRISHTI_STREAM_LOG_FUNC(9,4,m_streamLogger);
        
        auto pDel = identity(model);

        cv::Mat features;
        cv::Mat(ftrResult.ftrs).reshape(1, 1).copyTo(features);
        features.convertTo(features, CV_32F);

        {
            // XGBOOST
            std::vector<std::vector<float>> data(features.rows);
            for(int i = 0; i < features.rows; i++)
            {
                data[i] = features.row(i);
            }

            for(auto &t : reg.xgbdt)
            {
                DRISHTI_STREAM_LOG_FUNC(9,5,m_streamLogger);
                pDel[t.first] = (*t.second)(data[0]);
                DRISHTI_STREAM_LOG_FUNC(9,6,m_streamLogger);
            }
        }

        //std::cout << "****" << phiToEllipse(pIn) << "*****" << std::endl;
        //std::cout << "======== " << t << "=========== " << std::endl;
        //std::cout << "delta : " << phiToEllipse(pDel) << " "; print(pDel);
        //std::cout << "pIn : " << phiToEllipse(p) << " "; print(p);

        p = compose(model, p, pDel);
        result.pAll[t] = p; // store result for this stage

#if DRISHTI_CPR_DO_DEBUG && !HAS_XGBOOST
        // TODO: Legacy non xgboost
        if(doPreview)
        {
            cv::Mat canvas;
            cv::cvtColor(I, canvas, cv::COLOR_GRAY2BGR);

            const auto e = phiToEllipse(p), eIn = phiToEllipse(pIn);
            cv::ellipse(canvas, eIn, {255,0,0}, 1, 8);
            cv::ellipse(canvas, e, {0,255,0}, 1, 8);

            drishti::geometry::Ellipse e2(e);
            cv::line(canvas, e.center, e2.getMajorAxisPos(), {0,255,0}, 1, 8);
            cv::imshow("I", canvas); // opt
            cv::waitKey(0);
        }
#endif

    }
    return 0;
}

DRISHTI_RCPR_END
