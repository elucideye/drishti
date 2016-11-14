/*!
  @file   QtFaceMonitor.h
  @author David Hirvonen
  @brief Qt sample facemonitor 

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef QT_FACE_MONITOR_H
#define QT_FACE_MONITOR_H

#include "drishti/hci/drishti_hci.h"
#include "drishti/hci/FaceMonitor.h"

#include <opencv2/core/core.hpp>

#include <vector>

class QtFaceMonitor : public drishti::hci::FaceMonitor
{
public:
    QtFaceMonitor(const cv::Vec2d &range);
    virtual bool isValid(const cv::Point3f &position);
    virtual void grab(std::vector<cv::Mat4b> &frames);
protected:
    cv::Vec2d m_range;
    uint64_t m_frameCounter = 0;
    uint64_t m_stackCounter = 0;
};

#endif // QT_FACE_MONITOR_H
