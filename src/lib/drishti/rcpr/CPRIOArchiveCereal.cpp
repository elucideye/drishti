/*! -*-c++-*-
  @file   CPRIOArchiveCereal.cpp
  @author David Hirvonen 
  @brief  Implementation of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

// Include string definitions first for ANDROID platforms
#include "drishti/core/drishti_stdlib_string.h" // FIRST
#include "drishti/rcpr/CPR.h"
#include "drishti/rcpr/CPRIOArchive.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cvmat_cereal.h"

#include <acf/ACFField.h>

#include <cereal/cereal.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>
#include <cereal/types/vector.hpp> // required for XGBooster
#include <cereal/types/polymorphic.hpp>

#include <opencv2/core.hpp>

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(drishti::core::Field<cv::Mat>, cereal::specialization::member_serialize);
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(drishti::rcpr::CPR, cereal::specialization::member_serialize);
CEREAL_CLASS_VERSION(drishti::rcpr::CPR, 1);

DRISHTI_RCPR_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

typedef cereal::PortableBinaryOutputArchive OArchive;
template void CPR::Model::Parts::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::Model::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::CprPrm::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::RegModel::serialize<OArchive>(OArchive& ar, const unsigned int);
template void CPR::serialize<OArchive>(OArchive& ar, const unsigned int);
template void Recipe::serialize<OArchive>(OArchive& ar, const unsigned int);

typedef cereal::PortableBinaryInputArchive IArchive;
template void CPR::Model::Parts::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::Model::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::CprPrm::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::RegModel::serialize<IArchive>(IArchive& ar, const unsigned int);
template void CPR::serialize<IArchive>(IArchive& ar, const unsigned int);
template void Recipe::serialize<IArchive>(IArchive& ar, const unsigned int);

DRISHTI_RCPR_NAMESPACE_END
