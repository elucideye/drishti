#ifndef DRISHTI_ML_PCA_IMPL
#define DRISHTI_ML_PCA_IMPL

#include "drishti/ml/PCA.h"

DRISHTI_ML_NAMESPACE_BEGIN

template<class Archive>
void StandardizedPCA::Standardizer::serialize(Archive & ar, const unsigned int version)
{
    ar & mu;
    ar & sigma;
}

template<class Archive>
void StandardizedPCA::serialize(Archive & ar, const unsigned int version)
{
    ar & m_transform;
    ar & m_pca;

    if(Archive::is_loading::value)
    {
        init();
    }
}

DRISHTI_ML_NAMESPACE_END

#endif // DRISHTI_ML_PCA_IMPL
