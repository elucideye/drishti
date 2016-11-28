#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"

#include "Face.h"
#include "FaceImpl.h"

CEREAL_CLASS_VERSION(DRISHTI_FACE::FaceModel, 2);

DRISHTI_FACE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef cereal::PortableBinaryOutputArchive3 OArchive;
template void FaceModel::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef cereal::PortableBinaryInputArchive3 IArchive;
template void FaceModel::serialize<IArchive>(IArchive &ar, const unsigned int);

DRISHTI_EYE_NAMESPACE_END
