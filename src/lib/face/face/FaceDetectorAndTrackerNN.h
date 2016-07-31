/*!
  @file   FaceDetectorAndTrackerNN.h
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Declaration of simple nearest neighbor (noop) FaceDetectorAndTracker.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "face/FaceDetectorAndTrackerImpl.h"

#ifndef FACE_DETECTOR_AND_TRACKER_NN
#define FACE_DETECTOR_AND_TRACKER_NN 1

BEGIN_FACE_NAMESPACE

class TrackerNN : public FaceDetectorAndTracker::TrackImpl
{
public:
    TrackerNN();
    ~TrackerNN();
    virtual void initialize(const cv::Mat1b &image, const FaceModel &face);
    virtual bool update(const cv::Mat1b &image, FaceModel &face);
    virtual std::vector<cv::Point2f> getFeatures() const;
protected:

    virtual void initializeWithRegions(const cv::Mat1b &image, const std::vector<cv::Rect> &regions) {}
    DRISHTI_FACE::FaceModel m_face;
};

END_FACE_NAMESPACE

#endif // FACE_DETECTOR_AND_TRACKER_LK
