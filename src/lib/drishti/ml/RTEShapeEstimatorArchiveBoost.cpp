#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/ml/RTEShapeEstimatorImpl.h"
#include "drishti/core/drishti_cvmat_boost.h" 

//BOOST_CLASS_IMPLEMENTATION(_SHAPE_PREDICTOR, boost::serialization::object_class_info);
//BOOST_CLASS_TRACKING(_SHAPE_PREDICTOR, boost::serialization::track_always);

DRISHTI_ML_NAMESPACE_BEGIN

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void RTEShapeEstimator::serialize<OArchive>(OArchive &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<OArchive>(OArchive &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<OArchive>(OArchive &ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void RTEShapeEstimator::serialize<IArchive>(IArchive &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<IArchive>(IArchive &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<IArchive>(IArchive &ar, const unsigned int);

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
typedef boost::archive::text_oarchive OArchiveTXT;
template void RTEShapeEstimator::serialize<IArchive>(OArchiveTXT &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<OArchiveTXT>(OArchiveTXT &ar, const unsigned int);

typedef IArchiveTXT IArchiveTXT;
template void RTEShapeEstimator::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<IArchiveTXT>(IArchiveTXT &ar, const unsigned int);
#endif

DRISHTI_ML_NAMESPACE_END

#include "drishti/core/boost_serialize_common.h"
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator::Impl);
