/*!
  @file   PCA.h
  @author David Hirvonen
  @brief  Internal PCA class declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_PCA_h
#define drishtisdk_PCA_h

#include "drishti/core/drishti_core.h"
#include "drishti/ml/drishti_ml.h"

#include "drishti/core/serialization.h"// for export

#include <opencv2/core/core.hpp>

#include <memory>

DRISHTI_ML_NAMESPACE_BEGIN

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
        void serialize(Archive & ar, const unsigned int version);
#if 0
        {
            ar & mu;
            ar & sigma;
        }
#endif
    };

    StandardizedPCA(); // null constructor for file loading
    void compute(const cv::Mat &data, cv::Mat &projection, float retainedVariance);
    void compute(const cv::Mat &data, cv::Mat &projection, int maxComponents);
    void init();

    size_t getNumComponents() const;

    cv::Mat project(const cv::Mat &data, int n=0) const;
    cv::Mat backProject(const cv::Mat &projection) const;

    template<class Archive>
    void serialize(Archive & ar, const unsigned int version);
#if 0
    {
        ar & m_transform;
        ar & m_pca;

        if(Archive::is_loading::value)
        {
            init();
        }
    }
#endif

    Standardizer m_transform;
    std::shared_ptr<cv::PCA> m_pca;

    cv::Mat m_eT; // transposed eigenvectors
};

DRISHTI_ML_NAMESPACE_END

BOOST_CLASS_EXPORT_KEY(drishti::ml::StandardizedPCA);

#endif
