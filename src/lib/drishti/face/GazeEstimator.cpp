/*!
  @file   GazeEstimator.cpp
  @author David Hirvonen
  @brief  Internal implementation for a simple model based relative gaze estimation scheme.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/GazeEstimator.h"
#include "drishti/face/FaceIO.h"
#include "drishti/face/face_util.h"
#include "drishti/ml/XGBooster.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/geometry/motion.h"
#include "drishti/core/boost_serialize_common.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

BEGIN_FACE_NAMESPACE

static void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order);

class GazeEstimator::Impl
{
public:

    using PointPair = GazeEstimator::GazeEstimate;

    struct GazeMeasurement
    {
        cv::Point2f innerCornersMean;
        cv::Point2f irisCentersMean;

        // Ratio of segment lengths:
        float normalizedInnerSegmentRatio;

        // Opennness
        float opennessL;
        float opennessR;

        // Parabola fit of iris centers and inner corners:
        cv::Vec3f poly;

        // 3D position:
        cv::Point3f position;

        float iod = 0.f;

        int count = 1;

        void reset()
        {
            innerCornersMean = {0.f,0.f};
            irisCentersMean = {0.f,0.f};
            normalizedInnerSegmentRatio = 0.f;
            opennessL = 0.f;
            opennessR = 0.f;
            poly = 0.f;
            position = {0.f,0.f,0.f};
            count = 0;
            iod = 0;
        }

        std::vector<float> asVector() const
        {
            std::vector<float> features
            {
                normalizedInnerSegmentRatio,
                opennessL,
                opennessR,
                poly[0],
                poly[1],
                poly[2],
                position.x,
                position.y,
                position.z
            };
            return features;
        }

        double timetsamp;
        face::FaceModel faceNorm;
        face::FaceModel faceNative;
    };

    drishti::core::Field<cv::Point2f> groundTruth;

    Impl()
    {

    }

    Impl(const sensor::SensorModel &sensor) : m_sensor(sensor) {}

    static std::shared_ptr<GazeEstimator::Impl> construct(std::ifstream &is, bool nose, bool brows, bool crease)
    {
        std::shared_ptr<GazeEstimator::Impl> ptr = std::make_shared<GazeEstimator::Impl>();
        load_pba_z(is, ptr->forests);
        ptr->m_doNose = nose;
        ptr->m_doBrows = brows;
        ptr->m_doCrease = crease;
        return ptr;
    }

    void setSensorModel(const sensor::SensorModel &sensor)
    {
        m_sensor = sensor;
    }

    void setGroundTruth(const cv::Point2f &p)
    {
        groundTruth = p;
    }

    // Use the trained classifier on a normalized face image:
    cv::Point2f operator()(const FaceModel &faceIn)
    {
        CV_Assert(forests.size() == 2);
        std::vector<float> params = getParams(faceIn);
        cv::Point2f p(forests[1](params), forests[0](params));
        return p;
    }

    std::vector<float> getParams(const FaceModel &faceIn) const
    {
        FaceModel faceNorm = getNormalizedFace(faceIn).first;
        GazeMeasurement measurement = computeFeaturesOnNormalizedFace(faceNorm);
        measurement.position = get3DPosition(faceIn); // 3d position from face @ native resolution

        std::vector<float> params = DRISHTI_FACE::faceToVector(faceNorm, m_doNose, m_doBrows, m_doCrease);
        std::vector<float> features = measurement.asVector();
        std::copy(features.begin(), features.end(), std::back_inserter(params));

        return params;
    }

    void verbose(bool flag)
    {
        m_doVerbose = flag;
    }
    bool verbose() const
    {
        return m_doVerbose;
    }

    void doImageFeatures(bool flag)
    {
        m_doImageFeatures = flag;
    }
    bool doImageFeatures() const
    {
        return m_doImageFeatures;
    }

    // ===========

    PointPair get()
    {
        cv::Point2f gazeAbsolute, gazeRelative;
        if(m_measurement.has)
        {
            gazeAbsolute = estimateGaze(*m_measurement); // absolute estimate
            if(m_gazeAbsolutePrev.has)
            {
                // Note: could also run deltas on gaze stuff:
                gazeRelative = gazeAbsolute - (*m_gazeAbsolutePrev);
            }
        }
        PointPair result { gazeAbsolute, gazeRelative };
        return result;
    }

    cv::Point2f getSimpleDelta() const
    {
        cv::Point2f p0(m_measurementPrev->irisCentersMean - m_measurementPrev->innerCornersMean);
        cv::Point2f p1(m_measurement->irisCentersMean - m_measurement->innerCornersMean);
        return ((p1 * 100.f/m_measurement->iod) - (p0 * 100.f/m_measurementPrev->iod));
    }

    float getOpennessDelta() const
    {
        float openness0 = (m_measurementPrev->opennessL + m_measurementPrev->opennessR) * 0.5;
        float openness1 = (m_measurement->opennessL + m_measurement->opennessR) * 0.5f;
        return (openness1 - openness0);
    }

    void begin()
    {
        // Reset the current measurement only...
        m_measurement.clear();
        m_measurement->reset();

        m_gazeAbsolutePrev.clear();
        (*m_gazeAbsolutePrev) = {0.f, 0.f};
    }

    PointPair end()
    {
        GazeEstimate estimate;
        if(m_measurementPrev.has)
        {
            estimate.relative = getSimpleDelta();
            estimate.openness = getOpennessDelta();
        }
        m_measurementPrev = m_measurement;

        return estimate;
    }

    void reset()
    {
        m_measurement.clear();
    }

    cv::Point2f estimateGaze(const GazeMeasurement &measurement) const
    {
        cv::Point2f gaze(measurement.normalizedInnerSegmentRatio, measurement.poly(2));
        return gaze;
    }

    std::pair<FaceModel, cv::Matx33f> getNormalizedFace(const FaceModel &faceIn) const
    {
        // Do similarity normalization on iris centers::
        std::array<cv::Point2f,2> ptsA {{ faceIn.eyeFullR->irisEllipse.center, faceIn.eyeFullL->irisEllipse.center }};
        std::array<cv::Point2f,2> ptsB {{ {0.25, 0.25}, {0.75, 0.25} }};
        ptsB[0] *= m_targetSize.width;
        ptsB[1] *= m_targetSize.width;
        cv::Matx33f H = transformation::estimateSimilarity(ptsA, ptsB);
        DRISHTI_FACE::FaceModel face = H * faceIn;
        return std::make_pair(face, H);
    }

    // #### Perform relative gaze ###

    // Extract 3d position of nose bridge from full resolution face:
    cv::Point3f get3DPosition(const FaceModel &faceIn) const
    {
        std::array<cv::Point2f, 2> eyes {{ faceIn.eyeFullR->irisEllipse.center, faceIn.eyeFullL->irisEllipse.center }};
        return m_sensor.intrinsic().getDepth(eyes, 0.064);
    }

    GazeMeasurement computeFeaturesOnNormalizedFace(const FaceModel &face) const
    {
        GazeMeasurement measurement;

        // #### landmark means ####
        measurement.irisCentersMean = (face.eyeFullR->irisEllipse.center + face.eyeFullL->irisEllipse.center);
        measurement.innerCornersMean = (face.eyeFullR->getInnerCorner() + face.eyeFullL->getInnerCorner()) * 0.5f;
        measurement.iod = cv::norm(face.eyeFullR->irisEllipse.center - face.eyeFullL->irisEllipse.center);

        cv::Point2f rLower = drishti::geometry::pointMedian(face.eyeFullR->getLowerEyelid());
        cv::Point2f lLower = drishti::geometry::pointMedian(face.eyeFullL->getLowerEyelid());
        measurement.innerCornersMean = (rLower + lLower) * 0.5f;

        // ##### Horizontal gaze ####
        cv::Point2f limbusInnerR, limbusOuterR, irisCenterR, limbusInnerL, limbusOuterL, irisCenterL;
        face.eyeFullR->estimateIrisLandmarks(irisCenterR, limbusInnerR, limbusOuterR);
        face.eyeFullL->estimateIrisLandmarks(irisCenterL, limbusInnerL, limbusOuterL);

        float innerCornerToLimbusR = cv::norm(limbusInnerR - face.eyeFullR->getInnerCorner());
        float innerCornerToLimbusL = cv::norm(limbusInnerL - face.eyeFullL->getInnerCorner());
        float noseToIrisCenterR = cv::norm(irisCenterR - *face.noseTip);
        float noseToIrisCenterL = cv::norm(irisCenterL - *face.noseTip);
        innerCornerToLimbusR /= noseToIrisCenterR;
        innerCornerToLimbusL /= noseToIrisCenterL;

        // Return normalized difference of inner segments:
        float innerSegmentSum = (innerCornerToLimbusL + innerCornerToLimbusR);
        measurement.normalizedInnerSegmentRatio = (innerCornerToLimbusL - innerCornerToLimbusR) / innerSegmentSum;

        // #### Vertical gaze ####
        measurement.opennessL = face.eyeFullL->openness();
        measurement.opennessR = face.eyeFullR->openness();

        // Fit eye corner parabola:
        cv::Point2f outerCornerL = face.eyeFullL->getOuterCorner();
        cv::Point2f outerCornerR = face.eyeFullR->getOuterCorner();
        cv::Point2f innerCornerL = face.eyeFullL->getInnerCorner();
        cv::Point2f innerCornerR = face.eyeFullR->getInnerCorner();
        cv::Vec4f xs( irisCenterR.x, innerCornerR.x, innerCornerL.x, irisCenterL.x );
        cv::Vec4f ys( irisCenterR.y, innerCornerR.y, innerCornerL.y, irisCenterR.y );
        cv::Mat1f poly(3,1);
        polyfit(cv::Mat(xs), cv::Mat(ys), poly, 2);
        measurement.poly = {poly(0,0), poly(1,0), poly(2,0) };

        return measurement;
    }

    /*
     * Note: Here we allow for low reoslution input facce model and images, since this
     * will be a common use case for faster processing.  This is complemented with a
     * cv::Matx33f mapping the face to the full resolution image, so that the sensor
     * model can be applied for mapping to world coordinates.
     */

    PointPair operator()(const FaceModel &faceIn, const cv::Mat &I, const cv::Matx33f &Hup, double time)
    {
        PointPair gaze;

        auto result = getNormalizedFace(faceIn);
        const auto &face = result.first;
        const auto &H = result.second;

        GazeMeasurement measurement = computeFeaturesOnNormalizedFace(face);
        measurement.position = get3DPosition(Hup * faceIn);

        // Accumulate measurements:
        m_measurement.has = true;
        (*m_measurement) = measurement;

        cv::Point2f gazeVector;

        gaze = get(); // hand tuned gaze estimate

        if(m_doVerbose)
        {
            // (Optional) corresponding warped image:
            paint(face, I, H, measurement, gazeVector);
            std::cout << gaze.absolute << " vs " << (groundTruth.get() * 2.f) - cv::Point2f(1,1) << std::endl;
            cv::waitKey(0);
        }
        if(groundTruth.has)
        {
            groundTruth.clear();
        }

        return gaze;
    }

    template<class Archive> void serialize(Archive & ar, const unsigned int version)
    {
        ar & m_doNose;
        ar & m_doBrows;
        ar & m_doCrease;
        ar & forests;
    }

protected:

    void paint(const FaceModel &face, const cv::Mat &I, const cv::Matx33f &H, const GazeMeasurement &measurement, const cv::Point2f &t);

    drishti::core::Field<GazeMeasurement> m_measurement; // accumulator
    drishti::core::Field<GazeMeasurement> m_measurementPrev; // previous gaze position (reference)
    drishti::core::Field<cv::Point2f> m_gazeAbsolutePrev;
    drishti::core::Field<cv::Mat> m_imagePrev;

    float m_targetIOD = 0.5f;

    bool m_doImageFeatures = false;

    sensor::SensorModel m_sensor;
    cv::Size m_targetSize = {256,256};

    bool m_doVerbose = false;
    bool m_doNose = false;
    bool m_doBrows = false;
    bool m_doCrease = false;
    std::vector<drishti::ml::XGBooster> forests;
};

void GazeEstimator::Impl::paint(const FaceModel &face, const cv::Mat &I, const cv::Matx33f &H, const GazeMeasurement &measurement, const cv::Point2f &t)
{
    cv::Mat canvas;
    cv::warpAffine(I, canvas, H.get_minor<2,3>(0,0), m_targetSize);
    if(canvas.channels() == 1)
    {
        cv::cvtColor(canvas, canvas, cv::COLOR_GRAY2BGR);
    }

    if(groundTruth.has)
    {
        cv::Point p((1.0 - groundTruth->x) * canvas.cols, groundTruth->y * canvas.rows);
        cv::circle(canvas, p, 10, {0,255,0}, -1, 8);
    }

    static auto rainbow = drishti::core::makeRainbow();
    std::vector<cv::Point2f> curve(canvas.cols);
    for(int x = 0; x < canvas.cols; x++)
    {
        float y = measurement.poly[0] + measurement.poly[1] * x + measurement.poly[2] * (x*x);
        curve[x] = {float(x), y};
    }
    std::vector<std::vector<cv::Point>> drawing(1);
    std::copy(curve.begin(), curve.end(), std::back_inserter(drawing[0]));
    cv::polylines(canvas, drawing, false, {0,255,0}, 1, 8);

    auto &eyeL = (*face.eyeFullL);
    auto &eyeR = (*face.eyeFullR);
    cv::Point2f centerL = drishti::core::centroid(eyeL.eyelids);
    cv::Point2f centerR = drishti::core::centroid(eyeR.eyelids);

    const float iod = cv::norm(centerL - centerR);
    cv::Point2f u1(1,0), v1(0,1);
    cv::Point2f center = ((centerL + centerR) * 0.5f) - (v1 * iod * 0.25);
    cv::line(canvas, center-(v1*10.f), center+(v1*10.f), {255,255,100}, 2, 8);
    cv::line(canvas, center, center+(u1*measurement.normalizedInnerSegmentRatio*1000.f), {255,255,100}, 2, 8);

    cv::line(canvas, center, center+(t * 20.f), {0,255,0}, 1, 8);

    cv::Point2f limbusInnerR, limbusOuterR, irisCenterR, limbusInnerL, limbusOuterL, irisCenterL;
    eyeR.estimateIrisLandmarks(irisCenterR, limbusInnerR, limbusOuterR);
    eyeL.estimateIrisLandmarks(irisCenterL, limbusInnerL, limbusOuterL);

    std::vector< std::pair<cv::Vec3b,cv::Point2f>> landmarks
    {
        {rainbow[0], limbusInnerR},
        {rainbow[1], limbusOuterR},
        {rainbow[2], irisCenterR},
        {rainbow[0], limbusInnerL},
        {rainbow[1], limbusOuterL},
        {rainbow[2], irisCenterL},
        {rainbow[0], face.noseTip},
        {rainbow[3], eyeL.getInnerCorner()},
        {rainbow[3], eyeR.getInnerCorner()},
        {rainbow[4], eyeL.getOuterCorner()},
        {rainbow[4], eyeR.getOuterCorner()},
        {rainbow[5], centerR},
        {rainbow[5], centerL}
    };
    cv::Vec3b green(0,255,0);
    eyeR.draw(canvas,0,0, {0,255,0}, 1);
    eyeL.draw(canvas,0,0, {0,255,0}, 1);

    for(int i = 0; i < landmarks.size(); i++)
    {
        cv::circle(canvas, landmarks[i].second, 2, {255,255,255}, -1, 8);
        cv::circle(canvas, landmarks[i].second, 1, landmarks[i].first, -1, 8);
    }
    cv::imshow("gaze", canvas);
}

// ### gaze ###

void GazeEstimator::setGroundTruth(const cv::Point2f &p)
{
    m_pImpl->setGroundTruth(p);
}

GazeEstimator::GazeEstimate
GazeEstimator::operator()(const FaceModel &face, const cv::Mat &I, const cv::Matx33f &Hup, double time) const
{
    return (*m_pImpl)(face, I, Hup, time);
}

std::vector<float> GazeEstimator::getParams(const FaceModel &face) const
{
    return m_pImpl->getParams(face);
}

std::shared_ptr<GazeEstimator> GazeEstimator::construct(std::ifstream &is, bool nose, bool brows, bool crease)
{
    std::shared_ptr<GazeEstimator> ge = std::make_shared<GazeEstimator>();
    ge->m_pImpl = GazeEstimator::Impl::construct(is, nose, brows, crease);
    return ge;
}

std::shared_ptr<GazeEstimator> GazeEstimator::construct(const std::string &filename, bool nose, bool brows, bool crease)
{
    std::shared_ptr<GazeEstimator> ge = std::make_shared<GazeEstimator>();
    std::ifstream is(filename, std::ios::binary);
    CV_Assert(is);
    ge->m_pImpl = GazeEstimator::Impl::construct(is, nose, brows, crease);
    return ge;
}

GazeEstimator::GazeEstimator()
{
    m_pImpl = std::make_shared<Impl>();
}

GazeEstimator::GazeEstimator(const sensor::SensorModel &sensor)
{
    m_pImpl = std::make_shared<Impl>(sensor);
}

GazeEstimator::GazeEstimator(const sensor::SensorModel &sensor, const std::string &filename)
{
    load_pba_z(filename, m_pImpl);
    m_pImpl->setSensorModel(sensor);
}

GazeEstimator::GazeEstimator(const sensor::SensorModel &sensor, std::ifstream &is)
{
    load_pba_z(is, m_pImpl);
    m_pImpl->setSensorModel(sensor);
}

void GazeEstimator::setSensor(const sensor::SensorModel &sensor)
{
    m_pImpl->setSensorModel(sensor);
}

void GazeEstimator::verbose(bool flag)
{
    m_pImpl->verbose(flag);
}
bool GazeEstimator::verbose() const
{
    return m_pImpl->verbose();
}
void GazeEstimator::reset()
{
    m_pImpl->reset();
}
void GazeEstimator::begin()
{
    m_pImpl->begin();
}
GazeEstimator::GazeEstimate GazeEstimator::end()
{
    return m_pImpl->end();
}
void GazeEstimator::doImageFeatures(bool flag)
{
    m_pImpl->doImageFeatures(flag);
}
bool GazeEstimator::doImageFeatures() const
{
    return m_pImpl->doImageFeatures();
}

// Boost serialization:
template<class Archive> void GazeEstimator::serialize(Archive & ar, const unsigned int version)
{
    ar & m_pImpl;
}

// Utility

static void polyfit(const cv::Mat& src_x, const cv::Mat& src_y, cv::Mat& dst, int order)
{
    CV_Assert((src_x.rows>0)&&(src_y.rows>0)&&(src_x.cols==1)&&(src_y.cols==1)&&(dst.cols==1)&&(dst.rows==(order+1))&&(order>=1));
    cv::Mat X;
    X = cv::Mat::zeros(src_x.rows, order+1,CV_32FC1);
    cv::Mat copy;
    for(int i = 0; i <=order; i++)
    {
        copy = src_x.clone();
        cv::pow(copy,i,copy);
        cv::Mat M1 = X.col(i);
        copy.col(0).copyTo(M1);
    }
    cv::Mat X_t, X_inv;
    cv::transpose(X,X_t);
    cv::Mat temp = X_t*X;
    cv::Mat temp2;
    cv::invert (temp,temp2);
    cv::Mat temp3 = temp2*X_t;
    cv::Mat W = temp3*src_y;
    W.copyTo(dst);
}


END_FACE_NAMESPACE
