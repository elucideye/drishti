/*! -*-c++-*-
  @file   FaceDetectorAndTrackerImpl.cpp
  @author David Hirvonen
  @brief  Declaration of private iplementation for FaceDetectorAndTrackerImpl.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorAndTrackerImpl.h"

DRISHTI_FACE_NAMESPACE_BEGIN

FaceDetectorAndTracker::TrackImpl::TrackImpl() {}
FaceDetectorAndTracker::TrackImpl::~TrackImpl() {}

void FaceDetectorAndTracker::TrackImpl::initializeWithRegions(const cv::Mat1b& image, const std::vector<cv::Rect>& regions)
{
}

void FaceDetectorAndTracker::TrackImpl::initialize(const cv::Mat1b& image, const FaceModel& face)
{
    std::vector<cv::Rect> rois;

#define TRACK_EYE_REGIONS 1
#if TRACK_EYE_REGIONS
    // Get the eye regions
    cv::Rect2f eyeR, eyeL;
    face.getEyeRegions(eyeR, eyeL);
    if (eyeR.size().area() && eyeL.size().area())
    {
        rois = { eyeR, eyeL };
    }
#else
    rois = { getNoseBridge(face) };
#endif

    initializeWithRegions(image, rois);

    m_face = face;
    m_startTime = std::chrono::system_clock::now();
    m_isInitialized = true;
}

void drawEyes(const cv::Mat1b& image, const FaceModel& face)
{
    cv::Mat canvas;
    cv::cvtColor(image, canvas, cv::COLOR_GRAY2BGR);
    cv::rectangle(canvas, face.eyeFullR->roi, { 0, 255, 0 }, 1, 8);
    cv::rectangle(canvas, face.eyeFullL->roi, { 0, 255, 0 }, 1, 8);
    cv::imshow("update", canvas), cv::waitKey(0);
}

cv::Rect getNoseBridge(const FaceModel& face)
{
    const cv::Point2f cR = face.eyeFullR->getInnerCorner();
    const cv::Point2f cL = face.eyeFullL->getInnerCorner();
    const float span = cv::norm(cR - cL) * 0.5f;
    const cv::Point2f center = (cR + cL) * 0.5f, diag(span, span);
    cv::Rect roi(center - diag, center + diag);
    return roi;
}

DRISHTI_FACE_NAMESPACE_END
