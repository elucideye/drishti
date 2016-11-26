#include <opencv2/core.hpp>
#include "drishti/core/drishti_cv_boost.h"

#include "portable_binary_oarchive.hpp"
#include "portable_binary_iarchive.hpp"

#include "drishti/eye/Eye.h"
#include "drishti/eye/EyeImpl.h"

BOOST_CLASS_VERSION(DRISHTI_EYE::EyeModel, 1);

DRISHTI_EYE_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void EyeModel::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void EyeModel::serialize<IArchive>(IArchive &ar, const unsigned int);

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
typedef boost::archive::text_oarchive OArchiveTXT;
template void EyeModel::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void EyeModel::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
#endif

DRISHTI_EYE_NAMESPACE_END


