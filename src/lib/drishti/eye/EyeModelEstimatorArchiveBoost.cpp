#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/eye/EyeModelEstimatorImpl.h"

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
template void EyeModelEstimator::Impl::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
#endif

template void EyeModelEstimator::Impl::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
template void EyeModelEstimator::Impl::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<boost::archive::text_oarchive>(boost::archive::text_oarchive &ar, const unsigned int);

template void EyeModelEstimator::Impl::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<boost::archive::text_iarchive>(boost::archive::text_iarchive &ar, const unsigned int);
#endif

DRISHTI_EYE_NAMESPACE_END

#include "drishti/core/boost_serialize_common.h"


BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator::Impl);
