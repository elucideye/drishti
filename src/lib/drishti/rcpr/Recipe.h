/*! -*-c++-*-
  @file   Recipe.h
  @author David Hirvonen
  @brief  Declaration of parameter set for a single cascade of pose regression.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_rcpr_Recipe_h__
#define __drishti_rcpr_Recipe_h__

#include "drishti/rcpr/drishti_rcpr.h"
#include <iostream>
#include <vector>

DRISHTI_RCPR_NAMESPACE_BEGIN

// ##### Experimental ######
struct Recipe
{
    int maxLeafNodes = 5;
    int maxDepth = 4;
    int treesPerLevel = 500;
    int featurePoolSize = 400;
    int featureSampleSize = 40;
    double learningRate = 0.1;
    double dataSampleRatio = 0.5;
    bool doMask = false;
    double featureRadius = 1.66;
    double lambda = 0.1;
    bool useNPD = false;

    std::vector<int> paramIndex;

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version);

    void print(std::ostream& os)
    {
        os << "maxLeafNodes: " << maxLeafNodes << std::endl;
        os << "maxDepth: " << maxDepth << std::endl;
        os << "treesPerLevel: " << treesPerLevel << std::endl;
        os << "featurePoolSize: " << featurePoolSize << std::endl;
        os << "featureSampleSize: " << featureSampleSize << std::endl;
        os << "learningRate: " << learningRate << std::endl;
        os << "dataSampleRatio: " << dataSampleRatio << std::endl;
        os << "featureRadius: " << featureRadius << std::endl;
        os << "lambda: " << lambda << std::endl;
        os << "useNPD: " << useNPD << std::endl;
        os << "doMask: " << doMask << std::endl;
        os << "paramIndex: {";
        for (const auto& i : paramIndex)
        {
            os << i;
        }
        os << "}" << std::endl;
    }
};

DRISHTI_RCPR_NAMESPACE_END

#endif // __drishti_rcpr_Recipe_h__
