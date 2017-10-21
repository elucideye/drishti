/*! -*-c++-*-
  @file   CPRIOArchive.h
  @author David Hirvonen 
  @brief  Declaration of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_CPRIOArchive_h__
#define __drishti_rcpr_CPRIOArchive_h__

#include "drishti/rcpr/drishti_rcpr.h"
#include "drishti/rcpr/CPR.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/ThrowAssert.h"

DRISHTI_RCPR_NAMESPACE_BEGIN

// Boost serialization:
template <class Archive>
void CPR::Model::Parts::serialize(Archive& ar, const unsigned int version)
{
    ar& prn;
    ar& lks;
    ar& joint;
    ar& mus;
    ar& sigs;
    ar& wts;
}

template <class Archive>
void CPR::Model::serialize(Archive& ar, const unsigned int version)
{
    ar& parts;
}

template <class Archive>
void CPR::CprPrm::FtrPrm::serialize(Archive& ar, const unsigned int version)
{
    ar& type;
    ar& F;
    ar& radius;
    ar& nChn;
}

template <class Archive>
void Recipe::serialize(Archive& ar, const unsigned int version)
{
    ar& maxLeafNodes;
    ar& maxDepth;
    ar& treesPerLevel;
    ar& featurePoolSize;
    ar& featureSampleSize;
    ar& featureRadius;
    ar& learningRate;
    ar& dataSampleRatio;
    ar& useNPD;
    ar& doMask;
    ar& paramIndex;
    ar& lambda;
}

template <class Archive>
void CPR::CprPrm::serialize(Archive& ar, const unsigned int version)
{
    ar& model;
    ar& T;
    ar& L;
    ar& ftrPrm;
    ar& verbose;

    // New experimental features
    ar& cascadeRecipes;
}

template <class Archive>
void CPR::RegModel::Regs::FtrData::serialize(Archive& ar, const unsigned int version)
{
    ar& type;
    ar& F;
    ar& nChn;

#if DRISHTI_CPR_DO_HALF_FLOAT
    std::vector<PointHalf> xs_;
    if (Archive::is_loading::value)
    {
        ar& xs_;
        copy(xs_, (*xs));
    }
    else
    {
        copy((*xs), xs_);
        ar& xs_;
    }
#else
    ar& xs;
#endif

    ar& pids;
}

template <class Archive>
void CPR::RegModel::Regs::serialize(Archive& ar, const unsigned int version)
{
    ar& ftrData;
    ar& r;
    ar& xgbdt;
}

template <class Archive>
void CPR::RegModel::serialize(Archive& ar, const unsigned int version)
{
    ar& model;
    ar& pStar;
    ar& pDstr;
    ar& T;
    ar& regs;
    ar& pStar_;
}

template <class Archive>
void CPR::serialize(Archive& ar, const unsigned int version)
{
    drishti_throw_assert(version == 1, "Incorrect CPR archive format, please update models");

    ar& cprPrm;
    ar& regModel;
}

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_CPRIOArchive_h__
