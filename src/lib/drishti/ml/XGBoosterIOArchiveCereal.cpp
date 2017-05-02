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

CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(xgboost::wrapper::Booster, cereal::specialization::non_member_serialize);
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(xgboost::gbm::IGradBooster, cereal::specialization::non_member_serialize);
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(xgboost::gbm::GBTree, cereal::specialization::non_member_serialize);
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(xgboost::tree::RegTree, cereal::specialization::non_member_serialize);

CEREAL_REGISTER_TYPE(xgboost::wrapper::Booster);
CEREAL_REGISTER_TYPE(xgboost::gbm::GBTree);
CEREAL_REGISTER_POLYMORPHIC_RELATION(xgboost::gbm::IGradBooster, xgboost::gbm::GBTree);

CEREAL_REGISTER_TYPE(xgboost::gbm::GBLinear);
CEREAL_REGISTER_POLYMORPHIC_RELATION(xgboost::gbm::IGradBooster, xgboost::gbm::GBLinear);
CEREAL_SPECIALIZE_FOR_ALL_ARCHIVES(xgboost::gbm::GBLinear, cereal::specialization::non_member_serialize);

CEREAL_REGISTER_TYPE(xgboost::tree::RegTree);
CEREAL_REGISTER_POLYMORPHIC_RELATION(TreeModel32f, xgboost::tree::RegTree);

DRISHTI_BEGIN_NAMESPACE(xgboost)
DRISHTI_BEGIN_NAMESPACE(wrapper)

template <class Archive>
void serialize(Archive& ar, xgboost::wrapper::Booster& booster, const unsigned int version)
{
    auto& parent = dynamic_cast<xgboost::learner::BoostLearner&>(booster);

    // CEREAL_NOTE: To reduce complexity, cereal does not support raw pointers
    // (unlike boost), so we need to add some code for memory allocation during
    // loading below.
    ar& parent.mparam;
    ar& parent.name_obj_;
    ar& parent.name_gbm_;
    if (Archive::is_loading::value)
    {
        parent.gbm_ = gbm::CreateGradBooster(parent.name_gbm_.c_str());
        parent.obj_ = learner::CreateObjFunction(parent.name_obj_.c_str());
        ;
    }
    ar&* parent.gbm_;
    ar&* parent.obj_;
}

DRISHTI_END_NAMESPACE(wrapper)

DRISHTI_BEGIN_NAMESPACE(gbm)

template <class Archive>
void serialize(Archive& ar, xgboost::gbm::IGradBooster& gb, const unsigned int version)
{
    // Polymorphic relations should be handled normally, but we are adding cereal specific
    // serialization with non-member functions, so we need to handle the cast here:
    {
        auto* handle = dynamic_cast<xgboost::gbm::GBTree*>(&gb);
        if (handle != nullptr)
        {
            serialize(ar, *handle, version);
            return;
        }
    }

    {
        auto* handle = dynamic_cast<xgboost::gbm::GBLinear*>(&gb);
        if (handle != nullptr)
        {
            serialize(ar, *handle, version);
            return;
        }
    }
}

template <class Archive>
void serialize(Archive& ar, xgboost::gbm::GBTree& gbt, const unsigned int version)
{
    if (Archive::is_loading::value)
    {
        ar& gbt.mparam;

        gbt.trees.resize(gbt.mparam.num_trees);
        for (auto& tree : gbt.trees)
        {
            tree = new xgboost::tree::RegTree;
        }
        gbt.tree_info.resize(gbt.mparam.num_trees);
    }
    else
    {
        auto p = gbt.mparam;
        p.num_pbuffer = 0;
        ar& p;
    }

    for (auto& tree : gbt.trees)
    {
        ar&* tree;
    }

    ar& gbt.tree_info;
}

DRISHTI_END_NAMESPACE(gbm)

DRISHTI_BEGIN_NAMESPACE(tree)
template <typename Archive>
void serialize(Archive& ar, xgboost::tree::RegTree& tree, const unsigned int version)
{
    auto& super = dynamic_cast<TreeModel32f&>(tree);
    ar& super;
}
DRISHTI_END_NAMESPACE(tree)

DRISHTI_END_NAMESPACE(xgboost)

// Tree model:
typedef xgboost::tree::RTreeNodeStat RTreeNodeStat;
typedef xgboost::tree::TreeModel<bst_float, RTreeNodeStat> TreeModel;

// ##################################################################
// #################### portable_binary_*archive ####################
// ##################################################################

DRISHTI_ML_NAMESPACE_BEGIN

#if DRISHTI_BUILD_CEREAL_OUTPUT_ARCHIVES
typedef cereal::PortableBinaryOutputArchive OArchive;
template void XGBooster::serialize<OArchive>(OArchive& ar, const unsigned int);
template void XGBooster::Recipe::serialize<OArchive>(OArchive& ar, const unsigned int);
template void XGBooster::Impl::serialize<OArchive>(OArchive& ar, const unsigned int);
#endif

typedef cereal::PortableBinaryInputArchive IArchive;
template void XGBooster::serialize<IArchive>(IArchive& ar, const unsigned int);
template void XGBooster::Recipe::serialize<IArchive>(IArchive& ar, const unsigned int);
template void XGBooster::Impl::serialize<IArchive>(IArchive& ar, const unsigned int);

DRISHTI_ML_NAMESPACE_END
