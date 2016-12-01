#include "drishti/ml/drishti_ml.h"
#include "drishti/core/drishti_cvmat_boost.h"
#include "drishti/core/drishti_pca_boost.h"
#include "drishti/ml/RTEShapeEstimatorImpl.h"

typedef portable_binary_oarchive OArchive;
typedef portable_binary_iarchive IArchive;

//DRISHTI_BEGIN_NAMESPACE(boost)
//DRISHTI_BEGIN_NAMESPACE(serialization)
//#include "drishti/ml/shape_predictor_archive.h"
//#if !DRISHTI_BUILD_MIN_SIZE
//template void serialize(OArchive & ar, drishti::ml::fshape &g, const unsigned int version);
//template void serialize(OArchive & ar, dlib::vector<float,2> &g, const unsigned int version);
//template void serialize(OArchive & ar, drishti::ml::impl::split_feature &g, const unsigned int version);
//template void serialize(OArchive & ar, drishti::ml::impl::regression_tree &g, const unsigned int version);
//template void serialize(OArchive & ar, drishti::ml::shape_predictor &sp, const unsigned int version);
//#endif
//template void serialize(IArchive & ar, drishti::ml::fshape &g, const unsigned int version);
//template void serialize(IArchive & ar, dlib::vector<float,2> &g, const unsigned int version);
//template void serialize(IArchive & ar, drishti::ml::impl::split_feature &g, const unsigned int version);
//template void serialize(IArchive & ar, drishti::ml::impl::regression_tree &g, const unsigned int version);
//template void serialize(IArchive & ar, drishti::ml::shape_predictor &sp, const unsigned int version);
//DRISHTI_END_NAMESPACE(serialization) // namespace serialization
//DRISHTI_END_NAMESPACE(boost) // namespace boost

std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(const std::string &filename)
{
    auto sp = std::make_shared<_SHAPE_PREDICTOR>();
    load_pba_z(filename, *sp);
    sp->populate_f16();
    return sp;
}

std::shared_ptr<_SHAPE_PREDICTOR> load_pba_z(std::istream &is)
{
    auto sp = std::make_shared<_SHAPE_PREDICTOR>();
    load_pba_z(is, *sp);
    sp->populate_f16();
    return sp;
}


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

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator::Impl);
