/*! -*-c++-*-
  @file   PCA.cpp
  @author David Hirvonen
  @brief  Internal PCA class implementation.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/PCA.h"
#include "drishti/core/make_unique.h"

#include <Eigen/Dense>

DRISHTI_ML_NAMESPACE_BEGIN

// ########## Scaling params ############

StandardizedPCA::Standardizer::Standardizer() = default;
StandardizedPCA::Standardizer::~Standardizer() = default;
StandardizedPCA::Standardizer::Standardizer(int size, int type)
{
    create(size, type);
}

void StandardizedPCA::Standardizer::create(int size, int type)
{
    mu.create(1, size, type);
    sigma.create(1, size, type);
}

void StandardizedPCA::Standardizer::compute(const cv::Mat& src, const cv::Mat &columnWeights)
{
    create(src.cols, CV_32FC1);
    float *pMu = mu.ptr<float>(), *pSigma = sigma.ptr<float>();
    for (int i = 0; i < src.cols; i++, pMu++, pSigma++)
    {
        cv::Scalar mu, sigma;
        cv::Mat x = src.col(i);
        cv::meanStdDev(x, mu, sigma);
        pMu[0] = mu[0];
        pSigma[0] = sigma[0];
    }
    
    if(!columnWeights.empty())
    {
        CV_Assert(columnWeights.size().area() == sigma.size().area());
        cv::divide(sigma, (sigma.cols == columnWeights.cols) ? columnWeights : columnWeights.t(), sigma);
    }
}

cv::Mat StandardizedPCA::Standardizer::standardize(const cv::Mat& data) const
{
    cv::Mat tmp_sigma = cv::repeat(sigma, data.rows / sigma.rows, data.cols / sigma.cols);
    cv::Mat tmp_mean = cv::repeat(mu, data.rows / mu.rows, data.cols / mu.cols), tmp_data;
    int ctype = mu.type();
    if (data.type() != ctype || tmp_mean.data == mu.data)
    {
        data.convertTo(tmp_data, ctype);
        cv::subtract(tmp_data, tmp_mean, tmp_data);
        cv::divide(tmp_data, tmp_sigma, tmp_data);
    }
    else
    {
        cv::subtract(data, tmp_mean, tmp_mean);
        cv::divide(tmp_mean, tmp_sigma, tmp_data);
    }
    return tmp_data;
}

cv::Mat StandardizedPCA::Standardizer::unstandardize(const cv::Mat& data) const
{
    cv::Mat tmp_sigma = cv::repeat(sigma, data.rows / sigma.rows, data.cols / sigma.cols);
    cv::Mat tmp_mean = cv::repeat(mu, data.rows / mu.rows, data.cols / mu.cols), tmp_data;
    int ctype = mu.type();
    if (data.type() != ctype || tmp_mean.data == mu.data)
    {
        data.convertTo(tmp_data, ctype);
        cv::multiply(tmp_data, tmp_sigma, tmp_data);
        cv::add(tmp_data, tmp_mean, tmp_data);
    }
    else
    {
        cv::multiply(data, tmp_sigma, tmp_data);
        cv::add(tmp_data, tmp_mean, tmp_data);
    }
    return tmp_data;
}

// ########### ScalePCA #############

StandardizedPCA::StandardizedPCA() = default;
StandardizedPCA::~StandardizedPCA() = default;

size_t StandardizedPCA::getNumComponents() const
{
    return m_transform.mu.cols;
}

void StandardizedPCA::compute(const cv::Mat& data, cv::Mat& projection, float retainedVariance, const cv::Mat &columnWeights)
{
    cv::Mat mu;
    m_transform.compute(data, columnWeights);
    cv::Mat data_ = m_transform.standardize(data);
    m_pca = drishti::core::make_unique<cv::PCA>(data_, mu, cv::PCA::DATA_AS_ROW, retainedVariance);
    m_pca->project(data_, projection);

    init();
}

void StandardizedPCA::compute(const cv::Mat& data, cv::Mat& projection, int maxComponents, const cv::Mat &columnWeights)
{
    cv::Mat mu;
    m_transform.compute(data, columnWeights);
    cv::Mat data_ = m_transform.standardize(data);
    m_pca = drishti::core::make_unique<cv::PCA>(data_, mu, cv::PCA::DATA_AS_ROW, maxComponents);
    m_pca->project(data_, projection);

    init();
}

void StandardizedPCA::init()
{
    // Cache transposed vectors for faster multiplication:
    if (!m_pca->eigenvectors.empty())
    {
        m_eT = m_pca->eigenvectors.t();
    }
}

cv::Mat StandardizedPCA::project(const cv::Mat& samples, int n) const
{
    cv::Mat samples_ = m_transform.standardize(samples), projection;
    if (n > 0)
    {
        // Construct partial eigenvectors
        cv::Mat eigenvectors = m_pca->eigenvectors({ 0, n }, cv::Range::all());
        cv::Mat eigenvalues = m_pca->eigenvalues({ 0, n }, cv::Range::all());
        cv::Mat mean = m_pca->mean;

        cv::Mat data = samples_;
        CV_Assert(!mean.empty() && !eigenvectors.empty() && ((mean.rows == 1 && mean.cols == data.cols) || (mean.cols == 1 && mean.rows == data.rows)));
        cv::Mat tmp_data, tmp_mean = cv::repeat(mean, data.rows / mean.rows, data.cols / mean.cols);
        int ctype = mean.type();
        if (data.type() != ctype || tmp_mean.data == mean.data)
        {
            data.convertTo(tmp_data, ctype);
            cv::subtract(tmp_data, tmp_mean, tmp_data);
        }
        else
        {
            cv::subtract(data, tmp_mean, tmp_mean);
            tmp_data = tmp_mean;
        }
        if (mean.rows == 1)
        {
            cv::gemm(tmp_data, eigenvectors, 1, {}, 0, projection, cv::GEMM_2_T);
        }
        else
        {
            cv::gemm(eigenvectors, tmp_data, 1, {}, 0, projection, 0);
        }
    }
    else
    {
        m_pca->project(samples_, projection);
    }
    return projection;
}

void StandardizedPCA::gemm_transpose(const cv::Mat& A, const cv::Mat& Bt, cv::Mat& result)
{
    assert(A.type() == CV_32F);
    assert(A.isContinuous());
    assert(A.cols == Bt.cols);
    assert(Bt.type() == CV_32F);

    using DynamicStride = Eigen::Stride<Eigen::Dynamic, Eigen::Dynamic>;
    using MatrixRowMajor = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>;
    using MatrixColMajor = Eigen::Matrix<float, Eigen::Dynamic, Eigen::Dynamic, Eigen::ColMajor>;
    using MapMatrixRowMajor = Eigen::Map<MatrixRowMajor, Eigen::Unaligned>;
    using MapMatrixColMajor = Eigen::Map<MatrixColMajor, Eigen::Unaligned, DynamicStride>;

    result.create(A.rows, Bt.rows, CV_32F);
    MapMatrixRowMajor C1(result.ptr<float>(), result.rows, result.cols);
    MapMatrixRowMajor A1(const_cast<float*>(A.ptr<float>()), A.rows, A.cols);
    MapMatrixColMajor B1t(const_cast<float*>(Bt.ptr<float>()), /*n*/ Bt.cols, Bt.rows, DynamicStride(Bt.step1(), 1));
    C1 = A1 * B1t;
}

cv::Mat StandardizedPCA::backProject(const cv::Mat& projection) const
{
    cv::Mat result;
    int n = projection.cols;
    if (n != m_pca->eigenvectors.cols)
    {
        // Construct partial eigenvectors
        cv::Mat eigenvectors = m_pca->eigenvectors({ 0, n }, cv::Range::all());
        cv::Mat eigenvalues = m_pca->eigenvalues({ 0, n }, cv::Range::all());
        cv::Mat mean = m_pca->mean;

        cv::Mat data = projection;
        CV_Assert(!mean.empty() && !eigenvectors.empty() && ((mean.rows == 1 && eigenvectors.rows == data.cols) || (mean.cols == 1 && eigenvectors.rows == data.rows)));

        cv::Mat tmp_data, tmp_mean;
        if (data.type() != tmp_data.type())
        {
            data.convertTo(tmp_data, mean.type()); // 12%
        }
        else
        {
            tmp_data = data;
        }
        if (mean.rows == 1)
        {
            tmp_mean = cv::repeat(mean, data.rows, 1);

            // Eigen multiplication ( no copy )
            const cv::Mat& A = tmp_data;
            const cv::Mat& Bt = m_eT;
            gemm_transpose(A, Bt({ 0, Bt.rows }, { 0, n }), result);
        }
        else
        {
            tmp_mean = cv::repeat(mean, 1, data.cols);
            cv::gemm(eigenvectors, tmp_data, 1, tmp_mean, 1, result, cv::GEMM_1_T);
        }
    }
    else
    {
        result = m_pca->backProject(projection);
    }

    result = m_transform.unstandardize(result);

    return result;
}

DRISHTI_ML_NAMESPACE_END
