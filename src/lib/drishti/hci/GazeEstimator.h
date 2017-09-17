/*! -*-c++-*-
  @file   GazeEstimator.h
  @author David Hirvonen
  @brief  Internal declaration for a simple model based relative gaze estimation scheme.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_GazeEstimator_h__
#define __drishti_hci_GazeEstimator_h__

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
        GazeEstimate(const cv::Point2f& relative, float openness = 0.0)
            : relative(relative)
            , openness(openness)
        {
        }

        cv::Point2f relative;
        float openness = 0.f;
    };

    GazeEstimator();

    void reset();
    void begin();
    GazeEstimate end();

    GazeEstimate operator()(const face::FaceModel& face) const;

protected:
    std::shared_ptr<Impl> m_pImpl;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_GazeEstimator_h__
