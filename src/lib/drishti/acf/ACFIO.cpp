/*!
  @file   ACFIO.cpp
  @author David Hirvonen (C++ implementation)
  @brief  Implementation of deserialization routines for ACFIO mat models.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/acf/ACFIO.h"
#include "drishti/acf/ACF.h"

DRISHTI_ACF_NAMESPACE_BEGIN

#if DRISHTI_SERIALIZE_WITH_CVMATIO

int Detector::deserialize(const std::string &filename)
{
    return deserialize(filename.c_str());
}

int Detector::deserialize(const char *filename)
{
    MatlabIO matio;
    ParserNode<Detector> root_(filename, *this);
    ParserNode<Detector> && detector_ = root_.create("detector", (*this));
    return deserialize(detector_);
}

int Detector::deserialize(std::istream &is)
{
    ParserNode<Detector> root_(is, *this);
    ParserNode<Detector> && detector_ = root_.create("detector", (*this));
    return deserialize(detector_);
}

int Detector::deserialize(ParserNodeDetector &detector_)
{    
    {
        // Parse the classifier
        auto && clf_ = detector_.create("clf", this->clf);
        clf_.parse<decltype(clf.fids)>("fids", clf.fids);
        clf_.parse<decltype(clf.thrs)>("thrs", clf.thrs);
        clf_.parse<decltype(clf.child)>("child", clf.child);
        clf_.parse<decltype(clf.hs)>("hs", clf.hs);
        clf_.parse<decltype(clf.weights)>("weights", clf.weights);
        clf_.parse<decltype(clf.depth)>("depth", clf.depth);
        clf_.parse<decltype(clf.errs)>("errs", clf.errs);
        clf_.parse<decltype(clf.losses)>("losses", clf.losses);
        clf_.parse<decltype(clf.treeDepth)>("treeDepth", clf.treeDepth);

        // Store as transpose for column-major assumption:
        clf.fids = clf.fids.t();
        clf.thrs = clf.thrs.t();
        clf.child = clf.child.t();
        clf.hs = clf.hs.t();
        clf.weights = clf.weights.t();
        clf.depth = clf.depth.t();
    }

    {
        auto && opts_ = detector_.create("opts", opts);
        {
            // ====pPyramid====

            auto && pPyramid_ = opts_.create("pPyramid", opts_->pPyramid);
            {
                // ====pChns====
                auto && pChns_ = pPyramid_.create("pChns", (*pPyramid_)->pChns );
                pChns_.parse<Field<double>,decltype((*pChns_)->shrink)>("shrink", (*pChns_)->shrink);
                pChns_.parse<Field<double>,decltype((*pChns_)->complete)>("complete", (*pChns_)->complete);
                {
                    //====pColor====
                    auto && pColor_ = pChns_.create("pColor", (*pChns_)->pColor);
                    pColor_.parse<Field<double>,decltype( (*pColor_)->enabled)>("enabled", (*pColor_)->enabled);
                    pColor_.parse<Field<double>,decltype( (*pColor_)->smooth)>("smooth", (*pColor_)->smooth);
                    pColor_.parse<decltype((*pColor_)->colorSpace)>("colorSpace", (*pColor_)->colorSpace);
                }
                {
                    //====pGradMag====
                    auto && pGradMag_ = pChns_.create("pGradMag", (*pChns_)->pGradMag);
                    pGradMag_.parse<Field<double>,decltype((*pGradMag_)->enabled)>("enabled", (*pGradMag_)->enabled);
                    pGradMag_.parse<Field<double>,decltype( (*pGradMag_)->colorChn) >("colorChn", (*pGradMag_)->colorChn);
                    pGradMag_.parse<Field<double>,decltype((*pGradMag_)->normRad)>("normRad", (*pGradMag_)->normRad);
                    pGradMag_.parse<Field<double>,decltype((*pGradMag_)->normConst)>("normConst", (*pGradMag_)->normConst);
                    pGradMag_.parse<Field<double>,decltype((*pGradMag_)->full)>("full", (*pGradMag_)->full);
                }

                {
                    // ====pGradHist====
                    auto && pGradHist_ = pChns_.create("pGradHist", (*pChns_)->pGradHist);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->enabled)>("enabled", (*pGradHist_)->enabled);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->binSize)>("binSize", (*pGradHist_)->binSize);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->nOrients)>("nOrients", (*pGradHist_)->nOrients);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->softBin)>("softBin", (*pGradHist_)->softBin);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->useHog)>("useHog", (*pGradHist_)->useHog);
                    pGradHist_.parse<Field<double>,decltype((*pGradHist_)->clipHog)>("clipHog", (*pGradHist_)->clipHog);
                }

                (*pChns_)->pCustom.set("pCustom", false, false); // TODO:
            }
            pPyramid_.parse<Field<double>, decltype((*pPyramid_)->nPerOct)>("nPerOct", (*pPyramid_)->nPerOct);
            pPyramid_.parse<Field<double>, decltype((*pPyramid_)->nOctUp)>("nOctUp", (*pPyramid_)->nOctUp);
            pPyramid_.parse<Field<double>, decltype((*pPyramid_)->nApprox)>("nApprox", (*pPyramid_)->nApprox);
            pPyramid_.parse<decltype((*pPyramid_)->lambdas)>("lambdas", (*pPyramid_)->lambdas);
            pPyramid_.parse<decltype((*pPyramid_)->pad)>("pad", (*pPyramid_)->pad);
            pPyramid_.parse<decltype((*pPyramid_)->minDs)>("minDs", (*pPyramid_)->minDs);
            pPyramid_.parse<decltype((*pPyramid_)->smooth)>("smooth", (*pPyramid_)->smooth);
            pPyramid_.parse<Field<double>, decltype((*pPyramid_)->concat)>("concat", (*pPyramid_)->concat);
            pPyramid_.parse<Field<double>, decltype((*pPyramid_)->complete)>("complete", (*pPyramid_)->complete);
        }

        opts_.parse<decltype(opts_->modelDs)>("modelDs", opts_->modelDs);
        opts_.parse<decltype(opts_->modelDsPad)>("modelDsPad", opts_->modelDsPad);

        {
            // ====pNms====
            auto && pNms_ = opts_.create("pNms", opts_->pNms);
            pNms_.parse<decltype((*pNms_)->type)>("type", (*pNms_)->type);
            pNms_.parse<decltype((*pNms_)->overlap)>("overlap", (*pNms_)->overlap);
            pNms_.parse<decltype((*pNms_)->ovrDnm)>("ovrDnm", (*pNms_)->ovrDnm);
        }

        opts_.parse<Field<double>,decltype(opts_->stride)>("stride", opts_->stride);
        opts_.parse<decltype(opts_->cascThr)>("cascThr", opts_->cascThr);
        opts_.parse<decltype(opts_->cascCal)>("cascCal", opts_->cascCal);
        opts_.parse<Field<std::vector<double>>, decltype(opts_->nWeak)>("nWeak", opts_->nWeak);

        {
            // ====pBoost====
            auto && pBoost_ = opts_.create("pBoost", opts_->pBoost);

            {
                // ====pTree====
                auto && pTree_ = pBoost_.create("pTree", (*pBoost_)->pTree);
                pTree_.parse<Field<double>,decltype((*pTree_)->nBins)>("nBins", (*pTree_)->nBins);
                pTree_.parse<Field<double>,decltype((*pTree_)->maxDepth)>("maxDepth", (*pTree_)->maxDepth);
                pTree_.parse<decltype((*pTree_)->minWeight)>("minWeight", (*pTree_)->minWeight);
                pTree_.parse<decltype((*pTree_)->fracFtrs)>("fracFtrs", (*pTree_)->fracFtrs);
                pTree_.parse<Field<double>,decltype((*pTree_)->nThreads)>("nThreads", (*pTree_)->nThreads);
            }

            pBoost_.parse<Field<double>,decltype((*pBoost_)->nWeak)>("nWeak", (*pBoost_)->nWeak);
            pBoost_.parse<Field<double>,decltype((*pBoost_)->discrete)>("discrete", (*pBoost_)->discrete);
            pBoost_.parse<Field<double>,decltype((*pBoost_)->verbose)>("verbose", (*pBoost_)->verbose);
        }

        opts_.parse<decltype(opts_->posGtDir)>("posGtDir", opts_->posGtDir);
        opts_.parse<decltype(opts_->posImgDir)>("posImgDir", opts_->posImgDir);
        opts_.parse<decltype(opts_->negImgDir)>("negImgDir", opts_->negImgDir);
        opts_.parse<decltype(opts_->posWinDir)>("posWinDir", opts_->posWinDir);
        opts_.parse<decltype(opts_->negWinDir)>("negWinDir", opts_->negWinDir);

        //"imreadp"
        //"pLoad"

        opts_.parse<Field<double>,decltype(opts_->nPos)>("nPos", opts_->nPos);
        opts_.parse<Field<double>,decltype(opts_->nNeg)>("nNeg", opts_->nNeg);
        opts_.parse<Field<double>,decltype(opts_->nPerNeg)>("nPerNeg", opts_->nPerNeg);
        opts_.parse<Field<double>,decltype(opts_->nAccNeg)>("nAccNeg", opts_->nAccNeg);

        {
            auto && pJitter_ = opts_.create("pJitter", opts_->pJitter);
            pJitter_.parse<Field<double>,decltype((*pJitter_)->flip)>("flip", (*pJitter_)->flip);
        }

        opts_.parse<Field<double>,decltype(opts_->winsSave)>("winsSave", opts_->winsSave);
    }

    return 0;
}
#else // DRISHTI_SERIALIZE_WITH_CVMATIO
int Detector::deserialize(const char *filename) { return -1; }
int Detector::deserialize(const std::string &filename) { return -1; }
int Detector::deserialize(std::istream &is) { return -1; }
int Detector::deserialize(ParserNodeDetector &detector_) { return -1; }
#endif // DRISHTI_SERIALIZE_WITH_CVMATIO

DRISHTI_ACF_NAMESPACE_END

#if DRISHTI_SERIALIZE_WITH_CEREAL
#  include "drishti/core/drishti_stdlib_string.h"
#  include "drishti/core/drishti_cereal_pba.h"
#endif

DRISHTI_ACF_NAMESPACE_BEGIN

int Detector::deserializeAny(const std::string &filename)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if(filename.find(".pba.z") != std::string::npos)
    {
        load_pba_z(filename, *this);
        return 0;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CEREAL
    if(filename.find(".cpb") != std::string::npos)
    {
        load_cpb(filename, *this);
        return 0;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CVMATIO
    if(filename.find(".mat") != std::string::npos)
    {
        return deserialize(filename);
    }
#endif
    return -1;
}
int Detector::deserializeAny(std::istream &is, const std::string &hint)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if((!hint.empty() && hint.find(".pba.z") != std::string::npos) || (hint.empty() && is_pba_z(is)))
    {
        load_pba_z(is, *this);
        return 0;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CEREAL
    if(hint.empty() || (hint.find(".cpb") != std::string::npos))
    {
        load_cpb(is, *this);
        return 0;
    }
#endif
#if DRISHTI_SERIALIZE_WITH_CVMATIO
    if(hint.empty() || (hint.find(".mat") != std::string::npos))
    {
        return deserialize(is);
    }
#endif
    return -1;
}

DRISHTI_ACF_NAMESPACE_END
