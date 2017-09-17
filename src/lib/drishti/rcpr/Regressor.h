/*! -*-c++-*-
  @file   Regressor.h
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Declaration of an abstract univariate regressor class/API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_Regressor_h__
#define __drishti_rcpr_Regressor_h__

#include "drishti/ml/XGBooster.h"

#include <vector>

DRISHTI_RCPR_NAMESPACE_BEGIN

class Regressor
{
public:
    template <typename T>
    using MatrixType = std::vector<std::vector<T>>;

    typedef std::vector<float> RealVector;
    typedef MatrixType<float> RealMatrix;
    typedef MatrixType<uint8_t> BoolMatrix;

    struct Recipe
    {
        int maxLeafNodes = 6; // approximately (tree-depth + 1)
        int trees = 500;
        double nu = 0.1;
        double oversamplingFactor = 20.0;
        int featurePoolSize = 400;
        double lambda = 0.1;
        double numTestSplits = 20;   // not used
        double featureRadius = 1.66; // feature_pool_region_padding

        template <class Archive>
        void serialize(Archive& ar, const unsigned int version)
        {
            ar& maxLeafNodes;
            ar& trees;
            ar& nu;
            ar& featurePoolSize;
            ar& lambda;
            ar& featureRadius;
        }
    };

    Regressor() {}

    // Evaluate:
    virtual float operator()(const RealVector& features) = 0;

    // Train:
    virtual void train(const MatrixType<float>& values, const RealVector& labels, const MatrixType<uint8_t>& mask = {}) = 0;
};

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_Regressor_h__
