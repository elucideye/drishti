/*!
  @file   XGBooster.cpp
  @author David Hirvonen
  @brief  Internal implementation of the XGBoost C++ interface class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/XGBooster.h"

// implementations in ctypes
#define _CRT_SECURE_NO_WARNINGS
#define _CRT_SECURE_NO_DEPRECATE
#include <cstdio>
#include <vector>
#include <string>
#include <cstring>
#include <cmath>
#include <algorithm>

// include all std functions
using namespace std;
#include "wrapper/xgboost_wrapper.h"
#include "src/data.h"
#include "src/learner/learner-inl.hpp"
#include "src/io/io.h"
#include "src/utils/utils.h"
#include "src/utils/math.h"
#include "src/utils/group_data.h"
#include "src/io/simple_dmatrix-inl.hpp"

// Probably want to hide this stuff
#include "drishti/core/boost_serialize_common.h"
#include "drishti/core/Logger.h"

#include <random>
#include <iostream>

using namespace xgboost;
using namespace xgboost::io;

#include "src/gbm/gblinear-inl.hpp"
#include "src/gbm/gbtree-inl.hpp"
#include "src/learner/objective-inl.hpp"

// Use this definition with custom boost serialization XGBoost lib,
// else simply wrap standard XGBoost serialization with a boost API.
// Note: Setting this to 1 (if possible) will significantly reduce
// model size requirements
#define USE_XGBOOST_WITH_BOOST 1

#if XGBOOST_USE_BOOST

// Learner (IGradBooster):
BOOST_SERIALIZATION_ASSUME_ABSTRACT(xgboost::gbm::IGradBooster);
BOOST_CLASS_EXPORT_GUID(xgboost::gbm::IGradBooster, "IGradBooster");
BOOST_CLASS_EXPORT_GUID(xgboost::gbm::GBLinear, "GBLinear")
BOOST_CLASS_EXPORT_GUID(xgboost::gbm::GBTree, "GBTree")

// Tree model:
typedef xgboost::tree::RTreeNodeStat RTreeNodeStat;
typedef xgboost::tree::TreeModel<bst_float, RTreeNodeStat> TreeModel;
BOOST_SERIALIZATION_ASSUME_ABSTRACT(TreeModel);
BOOST_CLASS_EXPORT_GUID(TreeModel, "TreeModel");
BOOST_CLASS_EXPORT_GUID(xgboost::tree::RegTree, "RegTree");

// Loss function:
BOOST_SERIALIZATION_ASSUME_ABSTRACT(xgboost::learner::IObjFunction);
BOOST_CLASS_EXPORT_GUID(xgboost::learner::IObjFunction, "IObjFunction");
BOOST_CLASS_EXPORT_GUID(xgboost::learner::RegLossObj, "RegLossObj");

#endif

DRISHTI_BEGIN_NAMESPACE(xgboost)
DRISHTI_BEGIN_NAMESPACE(wrapper)

// booster wrapper class
class Booster: public learner::BoostLearner
{
public:
    explicit Booster(const std::vector<DataMatrix*>& mats = {})
    {
        this->silent = 1;
        this->init_model = false;

        if(mats.size())
        {
            this->SetCacheData(mats);
        }
    }
    inline const float *Pred(const DataMatrix &dmat, int option_mask, unsigned ntree_limit, bst_ulong *len)
    {
        DRISHTI_STREAM_LOG_FUNC(7,1,m_streamLogger);

        this->CheckInitModel();
        this->Predict(dmat, (option_mask&1) != 0, &this->preds_, ntree_limit, (option_mask&2) != 0);
        *len = static_cast<bst_ulong>(this->preds_.size());
        return BeginPtr(this->preds_);
    }
    inline void BoostOneIter(const DataMatrix &train, float *grad, float *hess, bst_ulong len)
    {
        DRISHTI_STREAM_LOG_FUNC(7,2,m_streamLogger);

        this->gpair_.resize(len);
        const bst_omp_uint ndata = static_cast<bst_omp_uint>(len);
        #pragma omp parallel for schedule(static)
        for (bst_omp_uint j = 0; j < ndata; ++j)
        {
            gpair_[j] = bst_gpair(grad[j], hess[j]);
        }
        gbm_->DoBoost(train.fmat(), this->FindBufferOffset(train), train.info.info, &gpair_);
    }
    inline void CheckInitModel(void)
    {
        if (!init_model)
        {
            this->InitModel();
            init_model = true;
        }
    }
    inline void LoadModel(const char *fname)
    {
#if !DRISHTI_BUILD_MIN_SIZE 
        learner::BoostLearner::LoadModel(fname);
        this->init_model = true;
#else
        CV_Assert(false);
#endif
    }
    inline void LoadModelFromBuffer(const void *buf, size_t size)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        utils::MemoryFixSizeBuffer fs((void*)buf, size);
        learner::BoostLearner::LoadModel(fs, true);
        this->init_model = true;
#else
        CV_Assert(false);
#endif
    }
    inline const char *GetModelRaw(bst_ulong *out_len)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        this->CheckInitModel();
        model_str.resize(0);
        utils::MemoryBufferStream fs(&model_str);
        learner::BoostLearner::SaveModel(fs, false);
        *out_len = static_cast<bst_ulong>(model_str.length());
        if (*out_len == 0)
        {
            return NULL;
        }
        else
        {
            return &model_str[0];
        }
#else
        CV_Assert(false);
        return nullptr;
#endif
    }

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version)
    {
#if USE_XGBOOST_WITH_BOOST
        ar & boost::serialization::base_object<learner::BoostLearner>(*this);
#else
        if (Archive::is_loading::value)
        {
            ar & model_str;
            LoadModelFromBuffer(&model_str[0], model_str.size());
        }
        else
        {
            bst_ulong length = 0;
            GetModelRaw(&length); // uses internal model_str
            ar & model_str;
        }
#endif
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        m_streamLogger = logger;
    }

    // temporal data to save model dump
    std::string model_str;

private:

    bool init_model;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

DRISHTI_END_NAMESPACE(wrapper)

std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const float *data, bst_ulong nrow, bst_ulong ncol, float  missing)
{
    bool nan_missing = utils::CheckNAN(missing);

    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple &mat = *p_mat;
    mat.info.info.num_row = nrow;
    mat.info.info.num_col = ncol;
    for (bst_ulong i = 0; i < nrow; ++i, data += ncol)
    {
        bst_ulong nelem = 0;
        for (bst_ulong j = 0; j < ncol; ++j)
        {
            if (utils::CheckNAN(data[j]))
            {
                utils::Check(nan_missing, "There are NAN in the matrix, however, you did not set missing=NAN");
            }
            else
            {
                if (nan_missing || data[j] != missing)
                {
                    mat.row_data_.push_back(RowBatch::Entry(bst_uint(j), data[j]));
                    ++nelem;
                }
            }
        }
        mat.row_ptr_.push_back(mat.row_ptr_.back() + nelem);
    }
    return p_mat;
}

std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const MatrixType<float> &data, bst_ulong nrow, bst_ulong ncol, float  missing)
{
    bool nan_missing = utils::CheckNAN(missing);

    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple &mat = *p_mat;
    mat.info.info.num_row = nrow;
    mat.info.info.num_col = ncol;
    for (bst_ulong i = 0; i < nrow; ++i)
    {
        bst_ulong nelem = 0;
        for (bst_ulong j = 0; j < ncol; ++j)
        {
            if (utils::CheckNAN(data[i][j]))
            {
                utils::Check(nan_missing, "There are NAN in the matrix, however, you did not set missing=NAN");
            }
            else
            {
                if (nan_missing || data[i][j] != missing)
                {
                    mat.row_data_.push_back(RowBatch::Entry(bst_uint(j), data[i][j]));
                    ++nelem;
                }
            }
        }
        mat.row_ptr_.push_back(mat.row_ptr_.back() + nelem);
    }
    return p_mat;
}

std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const MatrixType<float> &data, bst_ulong nrow, bst_ulong ncol, const MatrixType<uint8_t> &mask)
{
    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple &mat = *p_mat;
    mat.info.info.num_row = nrow;
    mat.info.info.num_col = ncol;
    for (bst_ulong i = 0; i < nrow; ++i)
    {
        bst_ulong nelem = 0;
        for (bst_ulong j = 0; j < ncol; ++j)
        {
            if (!mask.size() || mask[i][j])
            {
                mat.row_data_.push_back(RowBatch::Entry(bst_uint(j), data[i][j]));
                ++nelem;
            }
        }
        mat.row_ptr_.push_back(mat.row_ptr_.back() + nelem);
    }
    return p_mat;
}

std::shared_ptr<spdlog::logger> m_streamLogger;

DRISHTI_END_NAMESPACE(xgboost)

/// Begin the XGBooster interface class

_DRISHTI_ML_BEGIN

template <typename T> std::string xtos(const T &t)
{
    std::stringstream ss;
    ss << t;
    return ss.str();
}

class XGBooster::Impl
{
public:

    typedef XGBooster::Recipe Recipe;

    Impl()
    {
        m_recipe.numberOfTrees = 512;
        m_recipe.maxDepth = 4;
        m_recipe.learningRate = 0.1;
        m_recipe.dataSubsample = 0.5;
        m_recipe.regression = true;

        init();
    }
    Impl(const Recipe &recipe) : m_recipe(recipe)
    {
        init();
    }

    void init()
    {
        m_booster = std::make_shared<xgboost::wrapper::Booster>();

        //xgboost::wrapper::Booster tsk(dmats);

        // https://github.com/dmlc/xgboost/blob/master/doc/parameter.md
        m_booster->SetParam("silent", "1");
        m_booster->SetParam("nthread", "8");
        m_booster->SetParam("booster", "gbtree");

        m_booster->SetParam("eta", xtos(m_recipe.learningRate).c_str()); // shrinkage
        m_booster->SetParam("max_depth", xtos(m_recipe.maxDepth).c_str());
        m_booster->SetParam("subsample", xtos(m_recipe.dataSubsample).c_str());
        m_booster->SetParam("colsample_bytree", xtos(m_recipe.featureSubsample).c_str());

        m_booster->SetParam("gamma", "0");
        m_booster->SetParam("min_child_weight", "1"); // TODO (1 == dflt)
        m_booster->SetParam("max_delta_step", "0");   // TODO (1 == dflt)

        if(m_recipe.regression)
        {
            m_booster->SetParam("objective", "reg:linear"); // linear regression (vs. logistic)
        }
        else
        {
            m_booster->SetParam("objective", "binary:logistic");
        }
    }

    float operator()(const std::vector<float> &features)
    {
        std::shared_ptr<DMatrixSimple> dTest = xgboost::DMatrixSimpleFromMat(&features[0], 1, features.size(),NAN);
        std::vector<float> predictions(1, 0.f);
        m_booster->Predict(*dTest, false, &predictions);
        return predictions.front();
    }

    void train(const MatrixType<float> &features, const std::vector<float> &values, const MatrixType<uint8_t> &mask= {})
    {
#if !DRISHTI_BUILD_MIN_SIZE
        std::shared_ptr<DMatrixSimple> dTrain = xgboost::DMatrixSimpleFromMat(features, features.size(), features[0].size(), mask);
        dTrain->info.labels = values;

        std::vector< xgboost::learner::DMatrix *> dmats { dTrain.get() };
        m_booster->SetCacheData(dmats);
        m_booster->CheckInitModel();
        m_booster->CheckInit(dTrain.get());

        //std::vector<DataMatrix *> dmats { dTrain.get() }; // what is this for?

        for(int t = 0; t < m_recipe.numberOfTrees; t++)
        {
            m_booster->UpdateOneIter(t, *dTrain);
        }
#endif
    }

    void read(const std::string &name)
    {
#if !DRISHTI_BUILD_MIN_SIZE
        // normal XGBoost logging not needed with boost serialization
        m_booster->LoadModel(name.c_str());
#endif
    }

    void write(const std::string &name)
    {
#if !DRISHTI_BUILD_MIN_SIZE        
        m_booster->SaveModel(name.c_str(), true); // with_pbuffer TODO
#endif
    }

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version)
    {
        ar & m_recipe;
        ar & m_booster;
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        m_streamLogger = logger;
        if(m_booster)
        {
            m_booster->setStreamLogger(logger);
        }
    }

protected:

    Recipe m_recipe;
    std::shared_ptr<xgboost::wrapper::Booster> m_booster;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

XGBooster::XGBooster()
{
    m_impl = std::make_shared<XGBooster::Impl>();
}

XGBooster::XGBooster(const Recipe &recipe)
{
    m_impl = std::make_shared<XGBooster::Impl>(recipe);
}

void XGBooster::setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
{
    m_streamLogger = logger;
    if(m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

float XGBooster::operator()(const std::vector<float> &features)
{
    DRISHTI_STREAM_LOG_FUNC(7,3,m_streamLogger);
    return (*m_impl)(features);
}

void XGBooster::train(const MatrixType<float> &features, const std::vector<float> &values, const MatrixType<uint8_t> &mask)
{
#if !DRISHTI_BUILD_MIN_SIZE    
    m_impl->train(features, values, mask);
#endif
}

void XGBooster::read(const std::string &filename)
{
#if !DRISHTI_BUILD_MIN_SIZE     
    m_impl->read(filename);
#endif
}
void XGBooster::write(const std::string &filename) const
{
#if !DRISHTI_BUILD_MIN_SIZE 
    m_impl->write(filename);
#endif 
}

template<class Archive> void XGBooster::Recipe::serialize(Archive & ar, const unsigned int version)
{
    ar & numberOfTrees;
    ar & maxDepth;
    ar & dataSubsample;
    ar & learningRate;
    if(version >= 1)
    {
        ar & featureSubsample;
    }
}

// Boost serialization:
template<class Archive> void XGBooster::serialize(Archive & ar, const unsigned int version)
{
    ar & m_impl;
}

//#if !DRISHTI_BUILD_MIN_SIZE
template void XGBooster::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void XGBooster::Impl::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void XGBooster::Recipe::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
//#endif

template void XGBooster::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void XGBooster::Impl::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);
template void XGBooster::Recipe::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

_DRISHTI_ML_END

#if !DRISHTI_BUILD_MIN_SIZE
template void xgboost::tree::RegTree::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
#endif
template void xgboost::tree::RegTree::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::XGBooster);
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::XGBooster::Impl);
