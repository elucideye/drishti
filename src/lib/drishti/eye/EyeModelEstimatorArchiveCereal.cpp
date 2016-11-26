#include "drishti/core/drishti_cv_cereal.h"

#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/eye/EyeModelEstimatorImpl.h"

#include "drishti/core/drishti_cereal_pba.h"

// CEREAL_NOTE: We place this here since the ones in CPRIOArchiveCereal.cpp leads to missing symbosl
CEREAL_REGISTER_TYPE(drishti::rcpr::CPR);

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef cereal::PortableBinaryOutputArchive3 OArchive;
template void EyeModelEstimator::Impl::serialize<OArchive>(OArchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef cereal::PortableBinaryInputArchive3 IArchive;
template void EyeModelEstimator::Impl::serialize<IArchive>(IArchive &ar, const unsigned int);
template void EyeModelEstimator::serialize<IArchive>(IArchive &ar, const unsigned int);

DRISHTI_EYE_NAMESPACE_END
