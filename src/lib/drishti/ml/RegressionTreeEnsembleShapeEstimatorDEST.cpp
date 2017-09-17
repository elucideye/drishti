/*! -*-c++-*-
  @file   RegressionTreeEnsembleShapeEstimatorDEST.cpp
  @author David Hirvonen
  @brief  Internal implementation of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/ml/RegressionTreeEnsembleShapeEstimatorDEST.h"
#include "drishti/core/make_unique.h"

#include <dest/dest.h>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>

#include <array> // std::array<cv::Mat1f, 2>

DRISHTI_ML_NAMESPACE_BEGIN

/*
 * RegressionTreeEnsembleShapeEstimatorDEST::Impl
 */

class RegressionTreeEnsembleShapeEstimatorDEST::Impl
{

public:
    using Landmarks = std::vector<cv::Point2f>;

    Impl() {}

    Impl(const std::string& filename)
    {
        m_tracker.load(filename);
    }

    ~Impl()
    {
    }

    Impl(std::istream& is, const std::string& hint = {})
    {
        // TODO:
    }

    std::vector<cv::Point2f> operator()(const cv::Mat& gray)
    {
        dest::core::Rect r, ur = dest::core::unitRectangle();
        toDest(cv::Rect({ 0, 0 }, gray.size()), r);
        dest::core::ShapeTransform shapeToImage;
        shapeToImage = dest::core::estimateSimilarityTransform(ur, r);
        dest::core::MappedImage mappedGray = toDestHeaderOnly(gray);
        dest::core::Shape s = m_tracker.predict(mappedGray, shapeToImage);

        std::vector<cv::Point2f> landmarks(s.cols());
        for (int i = 0; i < s.cols(); i++)
        {
            landmarks[i] = { s(0, i), s(1, i) };
        }

        return landmarks;
    }

protected:
    // Convert OpenCV image to DEST reusing memory.
    inline dest::core::MappedImage toDestHeaderOnly(const cv::Mat& src)
    {
        eigen_assert(src.channels() == 1);
        eigen_assert(src.type() == CV_8UC1);
        const int outerStride = static_cast<int>(src.step[0] / sizeof(unsigned char));
        return dest::core::MappedImage(src.ptr<unsigned char>(), src.rows, src.cols, Eigen::OuterStride<Eigen::Dynamic>(outerStride));
    }

    //Convert OpenCV rectangle to DEST.
    inline void toDest(const cv::Rect& src, dest::core::Rect& dst)
    {
        dst = dest::core::createRectangle(Eigen::Vector2f(src.tl().x, src.tl().y), Eigen::Vector2f(src.br().x, src.br().y));
    }

    dest::core::Tracker m_tracker;
};

/*
 * RTEShapeEstimator
 */

RegressionTreeEnsembleShapeEstimatorDEST::RegressionTreeEnsembleShapeEstimatorDEST()
{
}

RegressionTreeEnsembleShapeEstimatorDEST::~RegressionTreeEnsembleShapeEstimatorDEST()
{
}

RegressionTreeEnsembleShapeEstimatorDEST::RegressionTreeEnsembleShapeEstimatorDEST(const std::string& filename)
{
    m_impl = drishti::core::make_unique<Impl>(filename);
}

RegressionTreeEnsembleShapeEstimatorDEST::RegressionTreeEnsembleShapeEstimatorDEST(std::istream& is, const std::string& hint)
{
    m_impl = drishti::core::make_unique<Impl>(is, hint);
}

// void RTEShapeEstimatorDEST::setStagesHint(int stages)
// {
//     m_impl->setStagesHint(stages);
// }

// int RTEShapeEstimatorDEST::getStagesHint() const
// {
//     return m_impl->getStagesHint();
// }

int RegressionTreeEnsembleShapeEstimatorDEST::operator()(const cv::Mat& gray, std::vector<cv::Point2f>& points, std::vector<bool>& mask) const
{
    points = (*m_impl)(gray);
    mask.resize(points.size());
    std::fill(begin(mask), end(mask), true);
    return 0;
}

int RegressionTreeEnsembleShapeEstimatorDEST::operator()(const cv::Mat& I, const cv::Mat& M, Point2fVec& points, BoolVec& mask) const
{
    CV_Assert(false);
    return 0;
}

bool RegressionTreeEnsembleShapeEstimatorDEST::isPCA() const
{
    return false;
}

std::vector<cv::Point2f> RegressionTreeEnsembleShapeEstimatorDEST::getMeanShape() const
{
    return std::vector<cv::Point2f>();
}

DRISHTI_ML_NAMESPACE_END
