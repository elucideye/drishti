/*! -*-c++-*-
 @file   Booster.h
 @author David Hirvonen
 @brief  Internal declaration of the XGBoost C++ interface class.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __drishti_ml_Booster_h__
#define __drishti_ml_Booster_h__

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
#include "xgboost/wrapper/xgboost_wrapper.h"
#include "xgboost/src/data.h"
#include "xgboost/src/io/io.h"
#include "xgboost/src/io/simple_dmatrix-inl.hpp" // DMatrixSimple
#include "xgboost/src/utils/utils.h"             // CheckNaN
#include "xgboost/src/learner/learner-inl.hpp"
#include "xgboost/src/utils/math.h"
#include "xgboost/src/utils/group_data.h"
#include "xgboost/src/gbm/gblinear-inl.hpp"
#include "xgboost/src/gbm/gbtree-inl.hpp"
#include "xgboost/src/learner/objective-inl.hpp"

#include "drishti/core/Logger.h"

#include <random>
#include <iostream>

using namespace xgboost;
using namespace xgboost::io;

// Use this definition with custom boost serialization XGBoost lib,
// else simply wrap standard XGBoost serialization with a boost API.
// Note: Setting this to 1 (if possible) will significantly reduce
// model size requirements
#define USE_XGBOOST_WITH_CEREAL 1

DRISHTI_BEGIN_NAMESPACE(xgboost)

inline std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const float* data, bst_ulong nrow, bst_ulong ncol, float missing)
{
    bool nan_missing = utils::CheckNAN(missing);

    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple& mat = *p_mat;
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

inline std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const MatrixType<float>& data, bst_ulong nrow, bst_ulong ncol, float missing)
{
#if DRISHTI_BUILD_MIN_SIZE
    assert(false);
    return std::shared_ptr<DMatrixSimple>();
#else
    bool nan_missing = utils::CheckNAN(missing);

    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple& mat = *p_mat;
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
#endif
}

inline std::shared_ptr<DMatrixSimple>
DMatrixSimpleFromMat(const MatrixType<float>& data, bst_ulong nrow, bst_ulong ncol, const MatrixType<uint8_t>& mask)
{
    std::shared_ptr<DMatrixSimple> p_mat = std::make_shared<DMatrixSimple>();
    DMatrixSimple& mat = *p_mat;
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

DRISHTI_BEGIN_NAMESPACE(wrapper)

// booster wrapper class
class Booster : public learner::BoostLearner
{
public:
    explicit Booster(const std::vector<DataMatrix*>& mats = {})
    {
        this->silent = 1;
        this->init_model = false;

        if (mats.size())
        {
            this->SetCacheData(mats);
        }
    }
    inline const float* Pred(const DataMatrix& dmat, int option_mask, unsigned ntree_limit, bst_ulong* len)
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
        return nullptr;
#else
        this->CheckInitModel();
        this->Predict(dmat, (option_mask & 1) != 0, &this->preds_, ntree_limit, (option_mask & 2) != 0);
        *len = static_cast<bst_ulong>(this->preds_.size());
        return BeginPtr(this->preds_);
#endif
    }
    inline void BoostOneIter(const DataMatrix& train, float* grad, float* hess, bst_ulong len)
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        this->gpair_.resize(len);
        const bst_omp_uint ndata = static_cast<bst_omp_uint>(len);
#pragma omp parallel for schedule(static)
        for (bst_omp_uint j = 0; j < ndata; ++j)
        {
            gpair_[j] = bst_gpair(grad[j], hess[j]);
        }
        gbm_->DoBoost(train.fmat(), this->FindBufferOffset(train), train.info.info, &gpair_);
#endif
    }
    inline void CheckInitModel(void)
    {
        if (!init_model)
        {
            this->InitModel();
            init_model = true;
        }
    }
    inline void LoadModel(const char* fname)
    {
        this->init_model = true;
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        learner::BoostLearner::LoadModel(fname);
#endif
    }
    inline void LoadModelFromBuffer(const void* buf, size_t size)
    {
        this->init_model = true;
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        utils::MemoryFixSizeBuffer fs((void*)buf, size);
        learner::BoostLearner::LoadModel(fs, true);
#endif
    }
    inline const char* GetModelRaw(bst_ulong* out_len)
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
        return nullptr;
#else
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
#endif
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
#if USE_XGBOOST_WITH_CEREAL
        auto& parent = dynamic_cast<xgboost::learner::BoostLearner&>(*this);
        ar& parent;
#else
        if (Archive::is_loading::value)
        {
            ar& model_str;
            LoadModelFromBuffer(&model_str[0], model_str.size());
        }
        else
        {
            bst_ulong length = 0;
            GetModelRaw(&length); // uses internal model_str
            ar& model_str;
        }
#endif
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
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
DRISHTI_END_NAMESPACE(xgboost)

#endif // __drishti_ml_Booster_h__
