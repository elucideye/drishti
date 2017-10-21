/*! -*-c++-*-
 @file   XGBoosterImpl.h
 @author David Hirvonen
 @brief  Internal declaration of the XGBoost private implementation class.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#ifndef __drishti_ml_XGBoosterImpl_h__
#define __drishti_ml_XGBoosterImpl_h__

#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/make_unique.h"
#include "drishti/core/ThrowAssert.h"
#include "drishti/ml/Booster.h"

DRISHTI_ML_NAMESPACE_BEGIN

template <typename T>
std::string xtos(const T& t)
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
    Impl(const Recipe& recipe)
        : m_recipe(recipe)
    {
        init();
    }

    ~Impl();

    void init()
    {
        m_booster = drishti::core::make_unique<xgboost::wrapper::Booster>();

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

        if (m_recipe.regression)
        {
            m_booster->SetParam("objective", "reg:linear"); // linear regression (vs. logistic)
        }
        else
        {
            m_booster->SetParam("objective", "binary:logistic");
        }
    }

    float operator()(const std::vector<float>& features)
    {
        std::shared_ptr<DMatrixSimple> dTest = xgboost::DMatrixSimpleFromMat(&features[0], 1, features.size(), NAN);
        std::vector<float> predictions(1, 0.f);
        m_booster->Predict(*dTest, false, &predictions);
        return predictions.front();
    }

    void train(const MatrixType<float>& features, const std::vector<float>& values, const MatrixType<uint8_t>& mask = {})
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        std::shared_ptr<DMatrixSimple> dTrain = xgboost::DMatrixSimpleFromMat(features, features.size(), features[0].size(), mask);
        dTrain->info.labels = values;

        std::vector<xgboost::learner::DMatrix*> dmats{ dTrain.get() };
        m_booster->SetCacheData(dmats);
        m_booster->CheckInitModel();
        m_booster->CheckInit(dTrain.get());

        for (int t = 0; t < m_recipe.numberOfTrees; t++)
        {
            m_booster->UpdateOneIter(t, *dTrain);
        }
#endif
    }

    void read(const std::string& name)
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        // normal XGBoost logging not needed with boost serialization
        m_booster->LoadModel(name.c_str());
#endif
    }

    void write(const std::string& name)
    {
#if DRISHTI_BUILD_MIN_SIZE
        assert(false);
#else
        m_booster->SaveModel(name.c_str(), true); // with_pbuffer TODO
#endif
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& m_recipe;
        ar& m_booster;
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        m_streamLogger = logger;
        if (m_booster)
        {
            m_booster->setStreamLogger(logger);
        }
    }

protected:
    Recipe m_recipe;
    std::unique_ptr<xgboost::wrapper::Booster> m_booster;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

// #################################################################################
// #########                      XGBooster                                #########
// #################################################################################

template <class Archive>
void XGBooster::Recipe::serialize(Archive& ar, const unsigned int version)
{
    ar& numberOfTrees;
    ar& maxDepth;
    ar& dataSubsample;
    ar& learningRate;
    ar& featureSubsample;
}

template <class Archive>
void XGBooster::serialize(Archive& ar, const unsigned int version)
{
    drishti_throw_assert(version == 1, "Incorrect XGBooster archive format, please update models");

    ar& m_impl;
}

DRISHTI_ML_NAMESPACE_END

#endif // __drishti_ml_XGBoosterImpl_h__
