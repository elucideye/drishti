/*! -*-c++-*-
  @file   RegressorXGBoost.h
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Declaration of an univariate XGBoost regressor.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_RegressorXGBoost_h__
#define __drishti_rcpr_RegressorXGBoost_h__

#include "drishti/rcpr/Regressor.h"
#include "drishti/ml/XGBooster.h"

#include <memory>
#include <vector>

DRISHTI_RCPR_NAMESPACE_BEGIN

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
    virtual float operator()(const RealVector& features)
    {
        return (*m_regressor)(features);
    }

    // Train:
    virtual void train(const MatrixType<float>& values, const RealVector& labels, const MatrixType<uint8_t>& mask = {})
    {
        m_regressor->train(values, labels, mask);
    }

    std::shared_ptr<ml::XGBooster> m_regressor;
};

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_RegressorXGBoost_h__
