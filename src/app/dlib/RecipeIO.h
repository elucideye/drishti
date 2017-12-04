/*! -*-c++-*-
  @file   RecipeIO.h
  @author David Hirvonen
  @brief  Declaration of parameter set for a cascaded pose regression (dlib).

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_dlib_RecipeIO_h__
#define __drishti_dlib_RecipeIO_h__

#include "drishti/core/drishti_core.h"

#include <vector>
#include <map>
#include <iostream>

DRISHTI_BEGIN_NAMESPACE(drishti)
DRISHTI_BEGIN_NAMESPACE(dlib)

struct Recipe
{
    bool do_affine = false;
    bool do_interpolate = false;
    bool npd = false;
    float lambda = 0.1;
    float nu = 0.1;
    float padding = 0;
    int cascades = 10;
    int depth = 2;
    int ellipse_count = 2;
    int features = 2048;
    int oversampling = 8;
    int splits = 512;
    int trees_per_level = 512;
    int width = 0;
    bool do_pca = true;
    std::vector<int> dimensions;
    
    // Sparse weights, assume 1.0 for all non missing entries:
    std::map<std::string,float> weights = { {"0", 1.0f}, {"8", 1.0f} };

    void print(std::ostream& os)
    {
        os << "cascade_depth: " << cascades << std::endl;
        os << "tree_depth: " << depth << std::endl;
        os << "num_trees_per_cascade_level: " << trees_per_level << std::endl;
        os << "nu: " << nu << std::endl;
        os << "oversampling_amount: " << oversampling << std::endl;
        os << "feature_pool_size: " << features << std::endl;
        os << "lambda: " << lambda << std::endl;
        os << "num_test_splits: " << splits << std::endl;
        os << "feature_pool_region_padding: " << padding << std::endl;
        os << "use npd: " << npd << std::endl;
        os << "affine: " << do_affine << std::endl;
        os << "interpolated: " << do_interpolate << std::endl;
    }
};

DRISHTI_END_NAMESPACE(dlib)
DRISHTI_END_NAMESPACE(drishti)

void loadJSON(const std::string& filename, drishti::dlib::Recipe& recipe);
void saveJSON(const std::string& filename, const drishti::dlib::Recipe& recipe);

#endif // __drishti_cpr_RecipeIO_h__
