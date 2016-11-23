/*!
  @file   CPRIOArchiveCereal.cpp
  @author David Hirvonen 
  @brief  Implementation of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#if 0

#include "drishti/rcpr/CPRIOArchive.h"
#include "drishti/core/drishti_cvmat_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"

#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>
#include <cereal/types/utility.hpp>

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(cv::Mat, cereal::specialization::non_member_load_save);

#if 0
DRISHTI_BEGIN_NAMESPACE(cv)
template<class Archive>
void save(Archive &ar, const cv::Point2f &p, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    drishti::rcpr::PointHalf q(p);
    ar & q.x;
    ar & q.y;
}

template<class Archive>
void load(Archive &ar, cv::Point2f &p, const std::uint32_t BOOST_ATTRIBUTE_UNUSED version)
{
    drishti::rcpr::PointHalf q;
    ar & q.x;
    ar & q.y;
    p = q;
}
DRISHTI_END_NAMESPACE(cv)
#endif

DRISHTI_RCPR_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef cereal::PortableBinaryOutputArchive3 OArchive;
template void CPR::Model::Parts::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::Model::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::RegModel::serialize<OArchive>(OArchive &ar, const unsigned int);
template void CPR::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef cereal::PortableBinaryInputArchive3 IArchive;
template void CPR::Model::Parts::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::Model::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::CprPrm::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::RegModel::serialize<IArchive>(IArchive &ar, const unsigned int);
template void CPR::serialize<IArchive>(IArchive &ar, const unsigned int);

DRISHTI_RCPR_NAMESPACE_END

#endif

