#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/RTEShapeEstimatorImpl.h"
#include "drishti/core/drishti_cvmat_cereal.h"
#include "drishti/core/drishti_pca_cereal.h"
#include "drishti/ml/PCAImpl.h"
#include "drishti/core/drishti_cereal_pba.h"

typedef cereal::PortableBinaryOutputArchive3 OArchive;
typedef cereal::PortableBinaryInputArchive3 IArchive;

DRISHTI_ML_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
template void RTEShapeEstimator::serialize<OArchive>(OArchive &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<OArchive>(OArchive &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<OArchive>(OArchive &ar, const unsigned int);
#endif
template void RTEShapeEstimator::serialize<IArchive>(IArchive &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<IArchive>(IArchive &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<IArchive>(IArchive &ar, const unsigned int);

DRISHTI_ML_NAMESPACE_END
