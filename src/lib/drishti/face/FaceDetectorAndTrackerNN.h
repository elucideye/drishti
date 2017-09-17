/*! -*-c++-*-
  @file   FaceDetectorAndTrackerNN.h
  @author David Hirvonen
  @brief  Declaration of simple nearest neighbor (noop) FaceDetectorAndTracker.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorAndTrackerImpl.h"

#ifndef __drishti_face_FaceDetectorAndTrackerNN_h__
#define __drishti_face_FaceDetectorAndTrackerNN_h__ 1

DRISHTI_FACE_NAMESPACE_BEGIN

class TrackerNN : public FaceDetectorAndTracker::TrackImpl
{
public:
    TrackerNN();
    ~TrackerNN();
    virtual void initialize(const cv::Mat1b& image, const FaceModel& face);
    virtual bool update(const cv::Mat1b& image, FaceModel& face);
    virtual std::vector<cv::Point2f> getFeatures() const;

protected:
    virtual void initializeWithRegions(const cv::Mat1b& image, const std::vector<cv::Rect>& regions) {}
    DRISHTI_FACE::FaceModel m_face;
};

DRISHTI_FACE_NAMESPACE_END

#endif // FACE_DETECTOR_AND_TRACKER_LK
