#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"

#include "Face.h"
#include "FaceImpl.h"

CEREAL_CLASS_VERSION(DRISHTI_FACE::FaceModel, 2);



// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################
DRISHTI_FACE_NAMESPACE_BEGIN
typedef cereal::PortableBinaryOutputArchive OArchive;
template void FaceModel::serialize<OArchive>(OArchive& ar, const unsigned int);
typedef cereal::PortableBinaryInputArchive IArchive;
template void FaceModel::serialize<IArchive>(IArchive& ar, const unsigned int);
DRISHTI_FACE_NAMESPACE_END

// ########################################
// ####### XML{Input,Output}Archive #######
// ########################################
#include <cereal/archives/xml.hpp>
DRISHTI_FACE_NAMESPACE_BEGIN
typedef cereal::XMLInputArchive IArchiveXML;
typedef cereal::XMLOutputArchive OArchiveXML;
template void FaceModel::serialize<OArchiveXML>(OArchiveXML& ar, const unsigned int);
template void FaceModel::serialize<IArchiveXML>(IArchiveXML& ar, const unsigned int);
DRISHTI_FACE_NAMESPACE_END

// ########################################
// ####### JSON{Input,Output}Archive #######
// ########################################
#include <cereal/archives/json.hpp>
DRISHTI_FACE_NAMESPACE_BEGIN
typedef cereal::JSONInputArchive IArchiveJSON;
typedef cereal::JSONOutputArchive OArchiveJSON;
template void FaceModel::serialize<OArchiveJSON>(OArchiveJSON& ar, const unsigned int);
template void FaceModel::serialize<IArchiveJSON>(IArchiveJSON& ar, const unsigned int);
DRISHTI_FACE_NAMESPACE_END
