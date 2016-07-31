/*!
  @file   RegressorXGBoost.h
  @author David Hirvonen (C++ implementation (gradient boosting trees)) <dhirvonen elucideye com>
  @brief  Declaration of an univariate XGBoost regressor.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef _DRISHTI_SDK_REGRESSOR_XGBOOST_
#define _DRISHTI_SDK_REGRESSOR_XGBOOST_

#include "rcpr/Regressor.h"
#include "ml/XGBooster.h"

#include <memory>
#include <vector>

DRISHTI_RCPR_BEGIN

class RegressorXGBoost : public Regressor
{
public:

    RegressorXGBoost()
    {
        ml::XGBooster::Recipe params;
        params.numberOfTrees = 1024;
        params.learningRate = 0.01;
        params.dataSubsample = 0.5;
        params.maxDepth = 2;
        m_regressor = std::make_shared<ml::XGBooster>(params);
    }

    // Evaluate:
    virtual float operator()(const RealVector &features)
    {
        return (*m_regressor)(features);
    }

    // Train:
    virtual void train(const MatrixType<float> &values, const RealVector &labels, const MatrixType<uint8_t> &mask= {})
    {
        m_regressor->train(values, labels, mask);
    }

    std::shared_ptr<ml::XGBooster> m_regressor;
};

DRISHTI_RCPR_END

#endif // _DRISHTI_SDK_REGRESSOR_XGBOOST_
