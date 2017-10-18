/*! -*-c++-*-
  @file   test-drishti-ml.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"
#include "drishti/ml/XGBooster.h"
#include "drishti/ml/PCA.h"

#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/drishti_cv_cereal.h"

TEST(XGBooster, XGBoosterInit)
{
    // Run simple function fit w/ full (non-lean) builds:
    drishti::ml::XGBooster booster;
    ASSERT_EQ(true, true);
}

TEST(StandardizedPCA, gemm_transpose_continuous)
{
    cv::Mat A, Bt, C;

    for (int i = 2; i < 6; i++)
    {
        int dim = (1 << i);

        // Simple:
        A = cv::Mat::ones(dim, dim, CV_32F);
        Bt = cv::Mat::ones(dim, dim, CV_32F);

        // Eigen runs plenty of numeric tests...
        // here we just run some simple sanity tests on matrix format/alignment:
        drishti::ml::StandardizedPCA::gemm_transpose(A, Bt, C);

        // Check size:
        ASSERT_EQ(C.rows, A.rows);
        ASSERT_EQ(C.cols, Bt.rows);

        for (auto iter = C.begin<float>(); iter != C.end<float>(); iter++)
        {
            EXPECT_FLOAT_EQ(*iter, float(dim));
        }
    }
}

TEST(StandardizedPCA, gemm_transpose_non_continuous)
{
    cv::Mat A, Bt1, Bt, C;

    for (int i = 2; i < 6; i++)
    {
        int dim = (1 << i);

        // Simple:
        A = cv::Mat::ones(dim, dim, CV_32F);
        Bt1 = cv::Mat::ones(dim + 8, dim + 8, CV_32F);
        Bt = Bt1({ 0, dim }, { 0, dim });

        // Eigen runs plenty of numeric tests...
        // here we just run some simple sanity tests on matrix format/alignment:
        drishti::ml::StandardizedPCA::gemm_transpose(A, Bt, C);

        // Check size:
        ASSERT_EQ(C.rows, A.rows);
        ASSERT_EQ(C.cols, Bt.rows);

        for (auto iter = C.begin<float>(); iter != C.end<float>(); iter++)
        {
            EXPECT_FLOAT_EQ(*iter, float(dim));
        }
    }
}
