#include <opencv2/core.hpp>
#include "drishti/core/drishti_cv_boost.h"

#include "pba/portable_binary_oarchive.hpp"
#include "pba/portable_binary_iarchive.hpp"

#include "drishti/eye/Eye.h"
#include "drishti/eye/EyeImpl.h"

BOOST_CLASS_VERSION(DRISHTI_EYE::EyeModel, 1);

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

DRISHTI_EYE_NAMESPACE_BEGIN

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void EyeModel::serialize<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void EyeModel::serialize<IArchive>(IArchive &ar, const unsigned int);
DRISHTI_EYE_NAMESPACE_END

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

DRISHTI_EYE_NAMESPACE_BEGIN
typedef boost::archive::text_oarchive OArchiveTXT;
template void EyeModel::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void EyeModel::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
DRISHTI_EYE_NAMESPACE_END
#endif




