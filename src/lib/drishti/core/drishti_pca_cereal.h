/*! -*-c++-*-
  @file   drishti_pca_cereal.h
  @author David Hirvonen
  @brief  Private header for cereal::serialize cv::Mat (de)serialization

*/

#ifndef __drishti_core_drishti_pca_cereal_h__
#define __drishti_core_drishti_pca_cereal_h__

#include "drishti/core/drishti_core.h" // for DRISHTI_BEGIN_NAMESPACE()
#include <opencv2/opencv.hpp>
#include <cereal/cereal.hpp>

DRISHTI_BEGIN_NAMESPACE(cv)
template <class Archive>
void serialize(Archive& ar, cv::PCA& pca, const unsigned int version)
{
    ar& pca.eigenvalues;
    ar& pca.eigenvectors;
    ar& pca.mean;
}
DRISHTI_END_NAMESPACE(cv)

#endif // __drishti_core_drishti_pca_cereal_h__
