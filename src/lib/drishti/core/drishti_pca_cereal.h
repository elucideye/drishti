/*!
  @file   drishti_pca_cereal.h
  @author David Hirvonen
  @brief  Private header for cereal::serialize cv::Mat (de)serialization

*/

#ifndef __DRISHTI_PCA_CEREAL_HPP_INCLUDED__
#define __DRISHTI_PCA_CEREAL_HPP_INCLUDED__

#include "drishti/core/drishti_core.h" // for DRISHTI_BEGIN_NAMESPACE()
#include <opencv2/opencv.hpp>
#include <cereal/cereal.hpp>

DRISHTI_BEGIN_NAMESPACE(cv)
template<class Archive>
void serialize(Archive & ar, cv::PCA &pca, const unsigned int version)
{
    ar & pca.eigenvalues;
    ar & pca.eigenvectors;
    ar & pca.mean;
}
DRISHTI_END_NAMESPACE(cv)

#endif // __DRISHTI_PCA_CEREAL_HPP_INCLUDED__
