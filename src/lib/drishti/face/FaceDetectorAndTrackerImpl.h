/*! -*-c++-*-
  @file   FaceDetectorAndTrackerImpl.h
  @author David Hirvonen
  @brief  Declaration of private iplementation for FaceDetectorAndTrackerImpl.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorAndTracker.h"

//#include <dlib/image_transforms/interpolation.h>
//#include <dlib/image_transforms/fhog.h>
//#include <dlib/image_processing/correlation_tracker.h>
//#include <dlib/opencv/cv_image.h>

#include <opencv2/features2d.hpp>
#include <opencv2/videostab/global_motion.hpp>
#include <opencv2/highgui.hpp>

#include <chrono>

#ifndef __drishti_face_FaceDetectorAndTrackerImpl_h__
#define __drishti_face_FaceDetectorAndTrackerImpl_h__ 1

DRISHTI_FACE_NAMESPACE_BEGIN

void drawEyes(const cv::Mat1b& image, const FaceModel& face);
cv::Rect getNoseBridge(const FaceModel& face);

// This really encapsulates the tracking specific stuff:
class FaceDetectorAndTracker::TrackImpl
{
public:
    TrackImpl();
    ~TrackImpl();

    /*
     * This will update cv::Rect eyeFull{L,R}::roi
     */

    virtual void initialize(const cv::Mat1b& image, const FaceModel& face);
    virtual bool update(const cv::Mat1b& image, FaceModel& face) = 0;
    virtual void reset()
    {
        m_isInitialized = false;
    }
    virtual bool hasTracks() const
    {
        return m_isInitialized;
    }

    virtual std::vector<cv::Point2f> getFeatures() const = 0;

    double trackAge() const
    {
        auto tic = std::chrono::system_clock::now();
        std::chrono::duration<double> elapsedSeconds = tic - m_startTime;
        return elapsedSeconds.count();
    }

    double getMaxTrackAge() const
    {
        return m_maxTrackAge;
    }
    void setMaxTrackAge(double age)
    {
        m_maxTrackAge = age;
    }

    void setFace(const FaceModel& face)
    {
        m_isInitialized = true;
        m_startTime = std::chrono::system_clock::now();
        m_face = face;
    }
    const FaceModel& getFace() const
    {
        return m_face;
    }

protected:
    virtual void initializeWithRegions(const cv::Mat1b& image, const std::vector<cv::Rect>& regions);

    FaceModel m_face;
    double m_maxTrackAge = 10000000000.0;
    bool m_isInitialized = false;

    std::chrono::time_point<std::chrono::system_clock> m_startTime;
};

DRISHTI_FACE_NAMESPACE_END

#endif // __drishti_face_FaceDetectorAndTrackerImpl_h__
