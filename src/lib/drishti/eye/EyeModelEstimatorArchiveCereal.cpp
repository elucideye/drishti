#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/eye/EyeModelEstimatorImpl.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"

CEREAL_CLASS_VERSION(DRISHTI_EYE::EyeModelEstimator, 1);

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

using OArchive = cereal::PortableBinaryOutputArchive;
template void EyeModelEstimator::Impl::serialize<OArchive>(OArchive& ar, const unsigned int);
template void EyeModelEstimator::serialize<OArchive>(OArchive& ar, const unsigned int);

using IArchive = cereal::PortableBinaryInputArchive;
template void EyeModelEstimator::Impl::serialize<IArchive>(IArchive& ar, const unsigned int);
template void EyeModelEstimator::serialize<IArchive>(IArchive& ar, const unsigned int);

DRISHTI_EYE_NAMESPACE_END
