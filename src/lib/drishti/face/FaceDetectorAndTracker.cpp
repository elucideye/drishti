/*! -*-c++-*-
  @file   FaceDetectorAndTracker.cpp
  @author David Hirvonen
  @brief  Implementation of a class that extends the face detector with tracking.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/face/FaceDetectorAndTracker.h"
#include "drishti/face/FaceDetectorAndTrackerImpl.h"
#include "drishti/face/FaceDetectorAndTrackerNN.h"

DRISHTI_FACE_NAMESPACE_BEGIN

// =============================

FaceDetectorAndTracker::FaceDetectorAndTracker(FaceDetectorFactory& resources)
    : FaceDetector(resources)
{
    //m_pImpl = std::make_shared<CorrelationTracker>();
    //m_pImpl = std::make_shared<LKTracker>();
    //m_pImpl = std::make_shared<CVTracker>();
    m_pImpl = std::make_shared<TrackerNN>();
}

std::vector<cv::Point2f> FaceDetectorAndTracker::getFeatures() const
{
    return m_pImpl->getFeatures();
}

void FaceDetectorAndTracker::setMaxTrackAge(double age)
{
    m_pImpl->setMaxTrackAge(age);
}
double FaceDetectorAndTracker::getMaxTrackAge() const
{
    return m_pImpl->getMaxTrackAge();
}

void FaceDetectorAndTracker::operator()(const MatP& I, const PaddedImage& Ib, std::vector<FaceModel>& faces, const cv::Matx33f& H)
{
    if (!m_pImpl->hasTracks() || (m_pImpl->trackAge() > m_pImpl->getMaxTrackAge()))
    {
        FaceDetector::operator()(I, Ib, faces, H); // do detection + regression
        if (faces.size())
        {
            m_pImpl->initialize(Ib.Ib, faces[0]); // initialize tracks
        }
    }
    else
    {
        bool okay = false;
        FaceModel face;
        bool trackOkay = m_pImpl->update(Ib.Ib, face); // here we have the left+right eyes
        if (trackOkay && face.roi->area())
        {
            okay = true;
        }
        else
        {
            m_pImpl->reset();
        }

        if (okay)
        {
            faces = { face };
            refine(Ib, faces, cv::Matx33f::eye(), false);
        }
    }
}

DRISHTI_FACE_NAMESPACE_END
