/*!
  @file   finder/FaceMonitor.h
  @author David Hirvonen
  @brief Simple position dependent frame grabbing callback API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef FACE_MONITOR_H
#define FACE_MONITOR_H

#include "drishti/hci/drishti_hci.h"

#include <opencv2/core/core.hpp>

#include <vector>

DRISHTI_HCI_NAMESPACE_BEGIN

class FaceMonitor
{
public:
    virtual bool isValid(const cv::Point3f &position) = 0;
    virtual void grab(std::vector<cv::Mat4b> &frames) = 0;
};

DRISHTI_HCI_NAMESPACE_END

#endif // FACE_MONITOR_H
