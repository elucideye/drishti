/*!
  @file   drishti_pca_boost.h
  @author David Hirvonen
  @brief  Private header for boost::serialize cv::Mat (de)serialization

*/

#ifndef __DRISHTI_PCA_BOOST_HPP_INCLUDED__
#define __DRISHTI_PCA_BOOST_HPP_INCLUDED__

#include "drishti/core/drishti_core.h" // for DRISHTI_BEGIN_NAMESPACE()
#include <opencv2/opencv.hpp>

#if 0
DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)

template<class Archive>
void serialize(Archive & ar, cv::PCA &pca, const unsigned int version)
{
    ar & pca.eigenvalues;
    ar & pca.eigenvectors;
    ar & pca.mean;
}

DRISHTI_END_NAMESPACE(serialization)
DRISHTI_END_NAMESPACE(boost)
#endif

#endif // __DRISHTI_PCA_BOOST_HPP_INCLUDED__
