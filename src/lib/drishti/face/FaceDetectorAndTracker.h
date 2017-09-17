/*! -*-c++-*-
  @file   FaceDetectorAndTracker.h
  @author David Hirvonen
  @brief  Declaration of a class that extends the face detector with tracking.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_face_FaceDetectorAndTracker_h__
#define __drishti_face_FaceDetectorAndTracker_h__

#include "drishti/face/FaceDetector.h"

DRISHTI_FACE_NAMESPACE_BEGIN

class FaceDetectorAndTracker : public FaceDetector
{
public:
    class TrackImpl;
    FaceDetectorAndTracker(FaceDetectorFactory& resources);
    virtual void operator()(const MatP& I, const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H);
    virtual std::vector<cv::Point2f> getFeatures() const;

    void setMaxTrackAge(double age);
    double getMaxTrackAge() const;

protected:
    std::shared_ptr<TrackImpl> m_pImpl; // make_unique fails
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceDetectorAndTracker_h__
