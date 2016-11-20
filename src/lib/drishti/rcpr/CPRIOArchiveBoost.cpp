/*!
  @file   CPRIOArchiveBoost.cpp
  @author David Hirvonen 
  @brief  Implementation of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/rcpr/CPRIOArchive.h"

BOOST_SERIALIZATION_SPLIT_FREE(cv::Point2f)

DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)

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

DRISHTI_END_NAMESPACE(serialization)
DRISHTI_END_NAMESPACE(boost)

//#if DRISHTI_CPR_DO_HALF_FLOAT
//    std::vector<PointHalf> xs_;
//    if(Archive::is_loading::value)
//    {
//        ar & xs_;
//        copy(xs_, (*xs));
//    }
//    else
//    {
//        copy((*xs), xs_);
//        ar & xs_;
//    }
//#else
//    ar & xs;
//#endif

DRISHTI_RCPR_NAMESPACE_BEGIN

// explicit instantiation:

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
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

typedef portable_binary_iarchive IArchive;
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

#if DRISHTI_USE_TEXT_ARCHIVES

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

typedef boost::archive::text_oarchive OArchiveTXT;
template void CPR::Model::Parts::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::Model::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void CPR::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void CPR::Model::Parts::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::Model::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FtrPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::FernPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::Recipe::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::CprPrm::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::FtrData::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::Regs::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::RegModel::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void CPR::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);

#endif // DRISHTI_USE_TEXT_ARCHIVES

DRISHTI_RCPR_NAMESPACE_END

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::rcpr::CPR);
