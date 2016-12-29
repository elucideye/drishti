/*!
  @file   drishti/hci/FaceMonitor.h
  @author David Hirvonen
  @brief Simple position dependent frame grabbing callback API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_hci_FaceMonitor_h__
#define __drishti_hci_FaceMonitor_h__

#include "drishti/hci/drishti_hci.h"
#include "drishti/face/Face.h"

#include <opencv2/core/core.hpp>

#include <vector> // std::vector<>
#include <chrono> // std::chrono::time_point

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceMonitor
{
public:
    
    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point;
    
    struct FaceImage
    {
        TimePoint time;
        cv::Mat4b image;
        cv::Mat4b eyes;
        cv::Mat4b extra; 
        std::vector<face::FaceModel> faces; // one or more faces
    };
    
    virtual bool isValid(const cv::Point3f &position, const TimePoint &timeStamp) = 0;
    virtual void grab(const std::vector<FaceImage> &frames, bool isInitialized) = 0;
};

DRISHTI_HCI_NAMESPACE_END

#endif // __drishti_hci_FaceMonitor_h__
