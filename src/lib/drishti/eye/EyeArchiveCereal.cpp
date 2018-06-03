#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cv_cereal.h"
#include "drishti/core/drishti_cereal_pba.h"

#include <cereal/archives/xml.hpp>
#include <cereal/archives/json.hpp>

#include "drishti/eye/Eye.h"
#include "drishti/eye/EyeImpl.h"

#include <fstream>

CEREAL_CLASS_VERSION(DRISHTI_EYE::EyeModel, 1);

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// ############ PortableBinary{Input,Output}Archive ################
// ##################################################################

using OArchive = cereal::PortableBinaryOutputArchive;
template void EyeModel::serialize<OArchive>(OArchive& ar, const unsigned int);

using IArchive = cereal::PortableBinaryInputArchive;
template void EyeModel::serialize<IArchive>(IArchive& ar, const unsigned int);

// #############################################
// #################### XML ####################
// #############################################

using OArchiveXML = cereal::XMLOutputArchive;
template void EyeModel::serialize<OArchiveXML>(OArchiveXML& ar, const unsigned int);

using IArchiveXML = cereal::XMLInputArchive;
template void EyeModel::serialize<IArchiveXML>(IArchiveXML& ar, const unsigned int);

// ##############################################
// #################### JSON ####################
// ##############################################

using OArchiveJSON = cereal::JSONOutputArchive;
template void EyeModel::serialize<OArchiveJSON>(OArchiveJSON& ar, const unsigned int);

using IArchiveJSON = cereal::JSONInputArchive;
template void EyeModel::serialize<IArchiveJSON>(IArchiveJSON& ar, const unsigned int);

DRISHTI_EYE_NAMESPACE_END
