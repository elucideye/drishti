/*!
  @file   GazeEstimator.h
  @author David Hirvonen
  @brief  Internal declaration for a simple model based relative gaze estimation scheme.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishtisdk__GazeEstimator__
#define __drishtisdk__GazeEstimator__

#include "drishti/hci/drishti_hci.h"
#include "drishti/face/Face.h"
#include "drishti/sensor/Sensor.h"

#include <memory>

DRISHTI_HCI_NAMESPACE_BEGIN

#define GAZE_NOSE 1
#define GAZE_BROW 1
#define GAZE_CREASE 1

class GazeEstimator
{
public:

    class Impl;

    struct GazeEstimate
    {
        GazeEstimate() {}
        GazeEstimate(const cv::Point2f &absolute, const cv::Point2f &relative, float openness=0.0)
            : absolute(absolute)
            , relative(relative)
            , openness(openness) {}

        cv::Point2f absolute;
        cv::Point2f relative;
        float openness;
    };

    // TODO: after this class is working, need to remove some of these
    GazeEstimator();
    GazeEstimator(const sensor::SensorModel &sensor);
    GazeEstimator(const sensor::SensorModel &sensor, std::ifstream &is);
    GazeEstimator(const sensor::SensorModel &sensor, const std::string &filename);

    // Create from regressor and params
    static std::shared_ptr<GazeEstimator> construct(std::ifstream &is, bool nose, bool brows, bool crease);
    static std::shared_ptr<GazeEstimator> construct(const std::string &is, bool nose, bool brows, bool crease);

    void setSensor(const sensor::SensorModel &sensor);

    void reset();
    void begin();
    GazeEstimate end();

    // Note: face models must be specified at native resolution of the sensor model:
    GazeEstimate operator()(const face::FaceModel &face, const cv::Mat &I, const cv::Matx33f &Hup, double time) const;

    void doImageFeatures(bool flag);
    bool doImageFeatures() const;

    void verbose(bool flag);
    bool verbose() const;

    void setGroundTruth(const cv::Point2f &p);

    std::vector<float> getParams(const face::FaceModel &face) const;

    // Boost serialization:
    template<class Archive> void serialize(Archive & ar, const unsigned int version);

protected:

    std::shared_ptr<Impl> m_pImpl;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishtisdk__GazeEstimator__
