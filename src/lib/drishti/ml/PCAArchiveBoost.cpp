#include "drishti/ml/PCA.h"
#include "drishti/ml/PCAImpl.h"

#include "drishti/core/drishti_serialization_boost.h"
#include "drishti/core/boost_serialize_common.h"
#include "drishti/core/drishti_cvmat_boost.h"

#include <boost/serialization/serialization.hpp>

DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)
template <class Archive>
void serialize(Archive& ar, cv::PCA& pca, const unsigned int version)
{
    ar& pca.eigenvalues;
    ar& pca.eigenvectors;
    ar& pca.mean;
}
DRISHTI_END_NAMESPACE(boost)
DRISHTI_END_NAMESPACE(serialization)

DRISHTI_ML_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void StandardizedPCA::serialize<OArchive>(OArchive& ar, const unsigned int);
template void StandardizedPCA::Standardizer::serialize(OArchive& ar, const unsigned int version);
#endif

typedef portable_binary_iarchive IArchive;
template void StandardizedPCA::serialize<IArchive>(IArchive& ar, const unsigned int);
template void StandardizedPCA::Standardizer::serialize(IArchive& ar, const unsigned int version);

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
typedef boost::archive::text_oarchive OArchiveTXT;
typedef boost::archive::text_iarchive IArchiveTXT;
template void StandardizedPCA::serialize<OArchiveTXT>(OArchiveTXT& ar, const unsigned int);
template void StandardizedPCA::serialize<IArchiveTXT>(IArchiveTXT& ar, const unsigned int);
#endif

DRISHTI_ML_NAMESPACE_END

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::StandardizedPCA);
