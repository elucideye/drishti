/*!
  @file   FaceDetectorAndTracker.h
  @author David Hirvonen
  @brief  Declaration of a class that extends the face detector with tracking.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __DRISHTI__FaceDetectorAndTracker__
#define __DRISHTI__FaceDetectorAndTracker__

#include "drishti/face/FaceDetector.h"

BEGIN_FACE_NAMESPACE

class FaceDetectorAndTracker : public FaceDetector
{
public:
    class TrackImpl;
    FaceDetectorAndTracker(FaceDetectorFactory &resources);
    virtual void operator()(const MatP &I, const PaddedImage &Ib, std::vector<FaceModel> &faces, const cv::Matx33f &H);
    virtual std::vector<cv::Point2f> getFeatures() const;

    void setMaxTrackAge(double age);
    double getMaxTrackAge() const;

protected:
    std::shared_ptr<TrackImpl> m_pImpl; // make_unique fails
};

END_FACE_NAMESPACE

#endif // __DRISHTI__FaceDetectorAndTracker__
