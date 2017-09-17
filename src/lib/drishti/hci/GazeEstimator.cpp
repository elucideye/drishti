/*! -*-c++-*-
  @file   GazeEstimator.cpp
  @author David Hirvonen
  @brief  Internal implementation for a simple model based relative gaze estimation scheme.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/hci/GazeEstimator.h"
#include "drishti/face/FaceIO.h"
#include "drishti/face/face_util.h"
#include "drishti/ml/XGBooster.h"
#include "drishti/geometry/Primitives.h"
#include "drishti/geometry/motion.h"

#include <opencv2/core/core.hpp>
#include <opencv2/video/video.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <fstream>

DRISHTI_HCI_NAMESPACE_BEGIN

class GazeEstimator::Impl
{
public:
    using PointPair = GazeEstimator::GazeEstimate;

    struct GazeMeasurement
    {
    };

    Impl()
    {
    }
    void begin()
    {
    }

    PointPair end()
    {
        return PointPair();
    }

    void reset()
    {
    }

    std::pair<face::FaceModel, cv::Matx33f> getNormalizedFace(const face::FaceModel& faceIn) const
    {
        // Do similarity normalization on iris centers::
        std::array<cv::Point2f, 2> ptsA{ { faceIn.eyeFullR->irisEllipse.center, faceIn.eyeFullL->irisEllipse.center } };
        std::array<cv::Point2f, 2> ptsB{ { { 0.25, 0.25 }, { 0.75, 0.25 } } };
        cv::Matx33f H = transformation::estimateSimilarity(ptsA, ptsB);
        DRISHTI_FACE::FaceModel face = H * faceIn;
        return std::make_pair(face, H);
    }

    PointPair operator()(const face::FaceModel& faceIn)
    {
        PointPair gaze;
        return gaze;
    }

protected:
};

// ### gaze ###

GazeEstimator::GazeEstimator()
{
    m_pImpl = std::make_shared<Impl>();
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

GazeEstimator::GazeEstimate GazeEstimator::operator()(const face::FaceModel& face) const
{
    return (*m_pImpl)(face);
}

DRISHTI_HCI_NAMESPACE_END
