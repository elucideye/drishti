#ifndef __drishti_ml_RTEShapeEstimatorImpl_h__
#define __drishti_ml_RTEShapeEstimatorImpl_h__

#include "drishti/ml/RegressionTreeEnsembleShapeEstimator.h"

// This type uses the multi-variate gradient boosted regression trees from dlib
// modified to support shape space model regression to reduce memory requirements.
// In addition, a few other options have been added such as normalized pixel
// differences and the line indexed features described in the RCPR publication.
// A half precision (16 bit) floating point representation is used to store
// the regression trees.

#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/core/make_unique.h"
#include "drishti/ml/drishti_ml.h"
#include "drishti/ml/shape_predictor_archive.h"

#define _SHAPE_PREDICTOR drishti::ml::shape_predictor

DRISHTI_ML_NAMESPACE_BEGIN

class RegressionTreeEnsembleShapeEstimator::Impl
{
public:
    Impl();
    Impl(const std::string& filename);
    Impl(std::istream& is, const std::string& hint = {});
    ~Impl();

    void packPointsInShape(const std::vector<cv::Point2f>& points, int ellipseCount, float* shape) const
    {
        // Copy initial chunk of 2d points
        int pointLength = points.size() - (ellipseCount * 5);
        memcpy(shape, &points[0].x, sizeof(float) * pointLength);

        // Next copy in the ellipses:
        for (int i = 0; i < ellipseCount; i++)
        {
            std::vector<float> phi = geometry::ellipseToPhi(geometry::pointsToEllipse(&points[pointLength + (i * 5)]));
            memcpy(&shape[pointLength * 2 + (i * 5)], &phi[0], sizeof(float) * phi.size());
        }
    }

    int operator()(const cv::Mat& crop, std::vector<cv::Point2f>& points, std::vector<bool>& mask) const
    {
        CV_Assert(crop.type() == CV_8UC1);

        auto& sp = *m_predictor;

        fshape initial_shape = sp.initial_shape;

        int paramCount = (points.size() * 2) - (m_predictor->m_ellipse_count * 5);
        if (paramCount == initial_shape.size())
        {
            packPointsInShape(points, m_predictor->m_ellipse_count, &initial_shape(0, 0));
        }

        // Zero copy cv::Mat wrapper:
        auto img = dlib::cv_image<uint8_t>(crop);
        dlib::rectangle roi(0, 0, crop.cols, crop.rows);
        dlib::full_object_detection shape = (*m_predictor)(img, roi, initial_shape, m_stagesHint);

        points.clear();
        points.reserve(initial_shape.size() / 2);
        for (int j = 0; j < shape.num_parts(); j++)
        {
            points.push_back(cv_point(shape.part(j)));
            mask.push_back(true);
        }

        return int(points.size());
    }

    void setStagesHint(int stages)
    {
        m_stagesHint = stages;
    }

    int getStagesHint() const
    {
        return m_stagesHint;
    }

    // {{p[0].x, p[0].y}, ..., {p[n].x,p[n.y}, {phi0[0],0}, {phi0[1],0} {phi0[2],0}, {phi0[3],0}, {phi0[4],0}}...
    std::vector<cv::Point2f> getMeanShape() const
    {
        std::vector<fpoint> data = convert_shape_to_points<float>(m_predictor->initial_shape, m_predictor->m_ellipse_count);
        std::vector<cv::Point2f> points(data.size());
        std::transform(data.begin(), data.end(), points.begin(), [](const fpoint& p) {
            return cv::Point2f(p.x(), p.y());
        });
        return points;
    }

    void dump(std::vector<float>& values, bool pca)
    {
        return m_predictor->getShapeUpdates(values, pca);
    }

    void setStreamLogger(std::shared_ptr<spdlog::logger>& logger)
    {
        m_streamLogger = logger;
        if (m_predictor)
        {
            m_predictor->setStreamLogger(logger);
        }
    }

    template <class Archive>
    void serialize(Archive& ar, const unsigned int version)
    {
        ar& m_predictor;

        if (Archive::is_loading::value)
        {
            m_predictor->populate_f16();
        }
    }

    bool isPCA() const
    {
        return bool(m_predictor->m_pca.get());
    }

    int m_inits = 1;
    int m_stagesHint = std::numeric_limits<int>::max();

    std::unique_ptr<_SHAPE_PREDICTOR> m_predictor;

    std::shared_ptr<spdlog::logger> m_streamLogger;
};

// Boost serialization:
template <class Archive>
void RTEShapeEstimator::serialize(Archive& ar, const unsigned int version)
{
    ar& m_impl;
}

template <class Archive>
void RTEShapeEstimator::serializeModel(Archive& ar, const unsigned int version)
{
    ar&(*m_impl->m_predictor);
}

DRISHTI_ML_NAMESPACE_END

#endif // __drishti_ml_RTEShapeEstimatorImpl_h__
