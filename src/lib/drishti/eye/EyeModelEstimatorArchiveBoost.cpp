#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/eye/EyeModelEstimatorImpl.h"

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void EyeModelEstimator::Impl::serialize<OArchive>(OArchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void EyeModelEstimator::Impl::serialize<IArchive>(IArchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<IArchive>(IArchive &ar, const unsigned int);

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
typedef boost::archive::text_oarchive OArchiveTXT;
template void EyeModelEstimator::Impl::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void EyeModelEstimator::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void EyeModelEstimator::Impl::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void EyeModelEstimator::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
#endif

DRISHTI_EYE_NAMESPACE_END

BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(DRISHTI_EYE::EyeModelEstimator::Impl);
