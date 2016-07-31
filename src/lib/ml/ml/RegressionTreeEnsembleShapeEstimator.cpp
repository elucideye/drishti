/*!
  @file   RegressionTreeEnsembleShapeEstimator.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Internal implementation of regression tree ensemble shape estimator variant.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "ml/RegressionTreeEnsembleShapeEstimator.h"

#include <dlib/image_transforms/assign_image.h>
#include <dlib/statistics/statistics.h>
#include <dlib/image_processing/shape_predictor.h>
#include <dlib/opencv/cv_image.h>
#include <dlib/vectorstream.h>
#include <dlib/serialize.h>

// This type uses the multi-variate gradient boosted regression trees from dlib
// modified to support shape space model regression to reduce memory requirements.
// In addition, a few other options have been added such as normalized pixel
// differences and the line indexed features described in the RCPR publication.
// A half precision (16 bit) floating point representation is used to store
// the regression trees.

#include "ml/shape_predictor.h"

#define _SHAPE_PREDICTOR drishti::ml::shape_predictor
//#define _SHAPE_PREDICTOR dlib

BOOST_CLASS_IMPLEMENTATION(_SHAPE_PREDICTOR, boost::serialization::object_class_info);
BOOST_CLASS_TRACKING(_SHAPE_PREDICTOR, boost::serialization::track_always);

_DRISHTI_ML_BEGIN

typedef RegressionTreeEnsembleShapeEstimator RTEShapeEstimator;

class RegressionTreeEnsembleShapeEstimator::Impl
{
public:

    Impl() {}

    Impl(const std::string &filename)
    {
        auto sp = std::make_shared<_SHAPE_PREDICTOR>();
        load_pba_z(filename, *sp);
        sp->populate_f16();
        m_predictor = sp;
    }

    void packPointsInShape(const std::vector<cv::Point2f> &points, int ellipseCount, float *shape) const
    {
        DRISHTI_STREAM_LOG_FUNC(6,2,m_streamLogger);
        // Copy initial chunk of 2d points
        int pointLength = points.size() - (ellipseCount*5);
        memcpy(shape, &points[0].x, sizeof(float)*pointLength);

        // Next copy in the ellipses:
        for(int i = 0; i < ellipseCount; i++)
        {
            std::vector<float> phi = geometry::ellipseToPhi(geometry::pointsToEllipse(&points[pointLength + (i * 5)]));
            memcpy(&shape[pointLength*2+(i*5)], &phi[0], sizeof(float)*phi.size());
        }
    }

    int operator()(const cv::Mat &crop, std::vector<cv::Point2f> &points, std::vector<bool> &mask) const
    {
        DRISHTI_STREAM_LOG_FUNC(6,3,m_streamLogger);
        CV_Assert(crop.type() == CV_8UC1);

        auto &sp = *m_predictor;

        fshape initial_shape = sp.initial_shape;

        int paramCount = (points.size() * 2) - (m_predictor->m_ellipse_count*5);
        if(paramCount == initial_shape.size())
        {
            packPointsInShape(points, m_predictor->m_ellipse_count, &initial_shape(0,0));
        }

        // std::cout << initial_shape - sp.initial_shape << " vs " << std::endl;

        // Zero copy cv::Mat wrapper:
        auto img = dlib::cv_image<uint8_t>(crop);
        dlib::full_object_detection shape = (*m_predictor)(img, dlib::rectangle(0,0,crop.cols,crop.rows), initial_shape, m_iters, m_stagesHint);

        points.clear();
        points.reserve(initial_shape.size()/2);
        for(int j = 0; j < shape.num_parts(); j++)
        {
            points.push_back(cv_point(shape.part(j)));
            mask.push_back( true );
        }

        return int(points.size());
    }

    void setStagesHint(int stages)
    {
        DRISHTI_STREAM_LOG_FUNC(6,4,m_streamLogger);
        m_stagesHint = stages;
    }

    int getStagesHint() const
    {
        DRISHTI_STREAM_LOG_FUNC(6,5,m_streamLogger);
        return m_stagesHint;
    }

    // {{p[0].x, p[0].y}, ..., {p[n].x,p[n.y}, {phi0[0],0}, {phi0[1],0} {phi0[2],0}, {phi0[3],0}, {phi0[4],0}}...
    std::vector<cv::Point2f> getMeanShape() const
    {
        DRISHTI_STREAM_LOG_FUNC(6,6,m_streamLogger);
        std::vector<fpoint> data = convert_shape_to_points<float>( m_predictor->initial_shape, m_predictor->m_ellipse_count);
        std::vector<cv::Point2f> points(data.size());
        std::transform(data.begin(), data.end(), points.begin(), [](const fpoint &p)
        {
            return cv::Point2f(p.x(), p.y());
        });
        return points;
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
    {
        m_streamLogger = logger;
        if(m_predictor)
        {
            m_predictor->setStreamLogger(logger);
        }
    }

    friend class boost::serialization::access;
    template<class Archive> void serialize(Archive & ar, const unsigned int version)
    {
        if (Archive::is_loading::value)
        {
            m_predictor = std::make_shared<_SHAPE_PREDICTOR>();
        }
        ar & (*m_predictor);
    }

    bool isPCA() const
    {
        return bool(m_predictor->m_pca.get());
    }

    int m_iters = 1;
    int m_inits = 1;
    int m_stagesHint = std::numeric_limits<int>::max();

    std::shared_ptr<_SHAPE_PREDICTOR> m_predictor; // TODO: Create virtual interface

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

void RTEShapeEstimator::setStreamLogger(std::shared_ptr<spdlog::logger> &logger)
{
    m_streamLogger = logger;
    if(m_impl)
    {
        m_impl->setStreamLogger(logger);
    }
}

RTEShapeEstimator::RegressionTreeEnsembleShapeEstimator(const std::string &filename)
{
    m_impl = std::make_shared<RegressionTreeEnsembleShapeEstimator::Impl>(filename);
}

void RTEShapeEstimator::setStagesHint(int stages)
{
    DRISHTI_STREAM_LOG_FUNC(6,7,m_streamLogger);
    m_impl->setStagesHint(stages);
}

int RTEShapeEstimator::getStagesHint() const
{
    DRISHTI_STREAM_LOG_FUNC(6,8,m_streamLogger);
    return m_impl->getStagesHint();
}

int RTEShapeEstimator::operator()(const cv::Mat &gray, std::vector<cv::Point2f> &points, std::vector<bool> &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6,9,m_streamLogger);
    return (*m_impl)(gray, points, mask);
}

int RTEShapeEstimator::operator()(const cv::Mat &I, const cv::Mat &M, Point2fVec &points, BoolVec &mask) const
{
    DRISHTI_STREAM_LOG_FUNC(6,10,m_streamLogger);
    CV_Assert(false);
    return 0;
}

bool RTEShapeEstimator::isPCA() const
{
    DRISHTI_STREAM_LOG_FUNC(6,11,m_streamLogger);
    return m_impl->isPCA();
}

std::vector<cv::Point2f> RTEShapeEstimator::getMeanShape() const
{
    return m_impl->getMeanShape();
}

template< class Archive> void RTEShapeEstimator::serializeModel(Archive &ar, const unsigned int version)
{
    ar & (*m_impl->m_predictor);
}

// Boost serialization:
template<class Archive> void
RTEShapeEstimator::serialize(Archive & ar, const unsigned int version)
{
    boost::serialization::void_cast_register<RTEShapeEstimator, ShapeEstimator>();
    ar & m_impl;
}

template void RTEShapeEstimator::serialize<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void RTEShapeEstimator::serialize<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

// ###### serialize model
template void RTEShapeEstimator::serializeModel<portable_binary_oarchive>(portable_binary_oarchive &ar, const unsigned int);
template void RTEShapeEstimator::serializeModel<portable_binary_iarchive>(portable_binary_iarchive &ar, const unsigned int);

_DRISHTI_ML_END

BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator);
BOOST_CLASS_EXPORT_IMPLEMENT(drishti::ml::RegressionTreeEnsembleShapeEstimator::Impl);
