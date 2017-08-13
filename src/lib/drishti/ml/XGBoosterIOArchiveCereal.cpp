#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"

#include "drishti/ml/XGBooster.h"
#include "drishti/ml/Booster.h"
#include "drishti/ml/XGBoosterImpl.h"

// include all std functions
using namespace std;
#include "xgboost/wrapper/xgboost_wrapper.h"
#include "xgboost/src/gbm/gbm.h"
#include "xgboost/src/data.h"
#include "xgboost/src/learner/learner-inl.hpp"
#include "xgboost/src/io/io.h"
#include "xgboost/src/utils/utils.h"
#include "xgboost/src/utils/math.h"
#include "xgboost/src/utils/group_data.h"
#include "xgboost/src/io/simple_dmatrix-inl.hpp"

#include <cereal/types/polymorphic.hpp>
#include <cereal/access.hpp>
#include <cereal/types/memory.hpp>

using namespace xgboost;
using namespace xgboost::io;

using TreeModel32f = xgboost::tree::TreeModel<bst_float, xgboost::tree::RTreeNodeStat>;

CEREAL_CLASS_VERSION(drishti::ml::XGBooster, 1);

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

typedef cereal::PortableBinaryOutputArchive OArchive;
typedef cereal::PortableBinaryInputArchive IArchive;

DRISHTI_ML_NAMESPACE_BEGIN
template void XGBooster::serialize<OArchive>(OArchive& ar, const unsigned int);
template void XGBooster::Recipe::serialize<OArchive>(OArchive& ar, const unsigned int);
template void XGBooster::Impl::serialize<OArchive>(OArchive& ar, const unsigned int);

template void XGBooster::serialize<IArchive>(IArchive& ar, const unsigned int);
template void XGBooster::Recipe::serialize<IArchive>(IArchive& ar, const unsigned int);
template void XGBooster::Impl::serialize<IArchive>(IArchive& ar, const unsigned int);
DRISHTI_ML_NAMESPACE_END

template void xgboost::gbm::IGradBooster::serialize(OArchive& ar, const unsigned int version);
template void xgboost::gbm::GBTree::serialize(OArchive& ar, const unsigned int version);
template void xgboost::gbm::GBLinear::serialize(OArchive& ar, const unsigned int version);
