#include "drishti/ml/drishti_ml.h"
#include "drishti/core/drishti_cvmat_boost.h"
#include "drishti/core/drishti_pca_boost.h"
#include "drishti/ml/RTEShapeEstimatorImpl.h"

// clang-format off
#if !DRISHTI_BUILD_MIN_SIZE
#  include "boost-pba/portable_binary_oarchive.hpp"
#endif
// clang-format on
#include "boost-pba/portable_binary_iarchive.hpp"

std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(const std::string& filename)
{
    auto sp = std::make_shared<_SHAPE_PREDICTOR>();
    load_pba_z(filename, *sp);
    sp->populate_f16();
    return sp;
}

std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(std::istream& is)
{
    auto sp = std::make_shared<_SHAPE_PREDICTOR>();
    load_pba_z(is, *sp);
    sp->populate_f16();
    return sp;
}

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

DRISHTI_ML_NAMESPACE_BEGIN

#if !DRISHTI_BUILD_MIN_SIZE
typedef portable_binary_oarchive OArchive;
template void RTEShapeEstimator::serialize<OArchive>(OArchive& ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<OArchive>(OArchive& ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<OArchive>(OArchive& ar, const unsigned int);
#endif

typedef portable_binary_iarchive IArchive;
template void RTEShapeEstimator::serialize<IArchive>(IArchive& ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<IArchive>(IArchive& ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<IArchive>(IArchive& ar, const unsigned int);

DRISHTI_ML_NAMESPACE_END

// ##################################################################
// #################### text_*archive ###############################
// ##################################################################

#if DRISHTI_USE_TEXT_ARCHIVES
#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

DRISHTI_ML_NAMESPACE_BEGIN
typedef boost::archive::text_oarchive OArchiveTXT;
template void RTEShapeEstimator::serialize<OArchiveTXT>(OArchiveTXT& ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<OArchiveTXT>(OArchiveTXT& ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<OArchiveTXT>(OArchiveTXT& ar, const unsigned int);

typedef boost::archive::text_iarchive IArchiveTXT;
template void RTEShapeEstimator::serialize<IArchiveTXT>(IArchiveTXT& ar, const unsigned int);
template void RTEShapeEstimator::Impl::serialize<IArchiveTXT>(IArchiveTXT& ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<IArchiveTXT>(IArchiveTXT& ar, const unsigned int);
DRISHTI_ML_NAMESPACE_END
#endif

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator::Impl);
