#include "drishti/core/drishti_core.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cvmat_cereal.h"
#include "drishti/core/drishti_pca_cereal.h"

typedef cereal::PortableBinaryOutputArchive OArchive;
typedef cereal::PortableBinaryInputArchive IArchive;

#include "drishti/ml/PCA.h"
#include "drishti/ml/PCAImpl.h"

#include <opencv2/core.hpp>

DRISHTI_ML_NAMESPACE_BEGIN
// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
template void StandardizedPCA::serialize<OArchive>(OArchive &ar, const unsigned int);
template void StandardizedPCA::Standardizer::serialize<OArchive>(OArchive & ar, const unsigned int version);
#endif
template void StandardizedPCA::serialize<IArchive>(IArchive &ar, const unsigned int);
template void StandardizedPCA::Standardizer::serialize<IArchive>(IArchive & ar, const unsigned int version);
DRISHTI_ML_NAMESPACE_END


//#if !DRISHTI_BUILD_MIN_SIZE
//template void StandardizedPCA::serialize<OArchive_>(OArchive_ &ar, const unsigned int);
//template void StandardizedPCA::Standardizer::serialize(OArchive_ & ar, const unsigned int version);
//#endif
//template void StandardizedPCA::serialize<IArchive_>(IArchive_ &ar, const unsigned int);
//template void StandardizedPCA::Standardizer::serialize(IArchive_ & ar, const unsigned int version);
//DRISHTI_ML_NAMESPACE_END

// ===

DRISHTI_BEGIN_NAMESPACE(cv)
template void serialize<OArchive>(OArchive &ar, cv::PCA &pca, const unsigned int);
template void serialize<IArchive>(IArchive &ar, cv::PCA &pca, const unsigned int);
DRISHTI_END_NAMESPACE(cv)
