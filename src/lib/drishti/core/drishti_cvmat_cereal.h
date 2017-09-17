/*! -*-c++-*-
  @file   drishti_cvmat_cereal.h
  @author Patrick Huber, etc
  @brief  Private header for cereal::serialize cv::Mat (de)serialization

*/

#ifndef __drishti_core_drishti_cvmat_cereal_h__
#define __drishti_core_drishti_cvmat_cereal_h__

#include "drishti/core/drishti_core.h" // for DRISHTI_BEGIN_NAMESPACE()
#include <opencv2/opencv.hpp>
#include <cereal/cereal.hpp>

DRISHTI_BEGIN_NAMESPACE(cv)

template <class Archive>
void save(Archive& ar, const cv::Mat& mat, const std::uint32_t version)
{
    int rows, cols, type;
    bool continuous;

    rows = mat.rows;
    cols = mat.cols;
    type = mat.type();
    continuous = mat.isContinuous();

    ar& rows& cols& type& continuous;

    if (continuous)
    {
        const int data_size = rows * cols * static_cast<int>(mat.elemSize());
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar& mat_data;
    }
    else
    {
        const int row_size = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++)
        {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar& row_data;
        }
    }
};

template <class Archive>
void load(Archive& ar, cv::Mat& mat, const std::uint32_t version)
{
    int rows, cols, type;
    bool continuous;

    ar& rows& cols& type& continuous;

    if (continuous)
    {
        mat.create(rows, cols, type);
        const int data_size = rows * cols * static_cast<int>(mat.elemSize());
        auto mat_data = cereal::binary_data(mat.ptr(), data_size);
        ar& mat_data;
    }
    else
    {
        mat.create(rows, cols, type);
        const int row_size = cols * static_cast<int>(mat.elemSize());
        for (int i = 0; i < rows; i++)
        {
            auto row_data = cereal::binary_data(mat.ptr(i), row_size);
            ar& row_data;
        }
    }
};

DRISHTI_END_NAMESPACE(cv)

#endif
