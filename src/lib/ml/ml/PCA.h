/*!
  @file   PCA.h
  @author David Hirvonen
  @brief  Internal PCA class declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_PCA_h
#define drishtisdk_PCA_h

#include "core/drishti_core.h"
#include "ml/drishti_ml.h"
#include "core/cvmat_serialization.h"

#include <opencv2/core/core.hpp>

#include <memory>

//# include <boost/archive/xml_oarchive.hpp>
//# define BOOST_NVP(name, value) boost::serialization::make_nvp(name, value)
#define BOOST_NVP(name, value) value

DRISHTI_BEGIN_NAMESPACE(boost)
DRISHTI_BEGIN_NAMESPACE(serialization)

template<class Archive>
void serialize(Archive & ar, cv::PCA &pca, const unsigned int version)
{
    ar & BOOST_NVP("eigenvalues", pca.eigenvalues);
    ar & BOOST_NVP("eigenvectors", pca.eigenvectors);
    ar & BOOST_NVP("mean", pca.mean);
}

DRISHTI_END_NAMESPACE(boost)
DRISHTI_END_NAMESPACE(serialization)

_DRISHTI_ML_BEGIN

class StandardizedPCA
{
public:

    struct Standardizer
    {
        Standardizer();
        Standardizer(int size, int type);
        void create(int size, int type);

        void compute(const cv::Mat &src);
        cv::Mat standardize(const cv::Mat &src) const;
        cv::Mat unstandardize(const cv::Mat &src) const;

        cv::Mat mu, sigma;

        template<class Archive>
        void serialize(Archive & ar, const unsigned int version)
        {
            ar & BOOST_NVP("mu", mu);
            ar & BOOST_NVP("sigma", sigma);
        }
    };

    StandardizedPCA(); // null constructor for file loading
    void compute(const cv::Mat &data, cv::Mat &projection, float retainedVariance);
    void compute(const cv::Mat &data, cv::Mat &projection, int maxComponents);
    void init();

    size_t getNumComponents() const;

    cv::Mat project(const cv::Mat &data, int n=0) const;
    cv::Mat backProject(const cv::Mat &projection) const;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version)
    {
        ar & BOOST_NVP("transform", m_transform);
        ar & BOOST_NVP("pca", m_pca);

        if(Archive::is_loading::value)
        {
            init();
        }
    }

    Standardizer m_transform;
    std::shared_ptr<cv::PCA> m_pca;

    cv::Mat m_eT; // transposed eigenvectors
};

_DRISHTI_ML_END

#endif
