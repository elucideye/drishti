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
#include "drishti/core/Field.h"

#include "thread_pool/thread_pool.hpp"

#include <opencv2/core/core.hpp>

#include <vector>

class QtFaceMonitor : public drishti::hci::FaceMonitor
{
public:
    
    struct PositionAndTime
    {
        PositionAndTime() : time(0.0) {}
        PositionAndTime(const cv::Point3f &position, double time) : position(position), time(time) {}
        double estimateVelocity(const PositionAndTime &current);
        double estimateVelocity(double position, double timeStamp);
        
        cv::Point3f position;
        double time;
    };
    
    QtFaceMonitor(const cv::Vec2d &range, std::shared_ptr<ThreadPool<128>> &threads);
    virtual bool isValid(const cv::Point3f &position, double timeStamp);
    virtual void grab(std::vector<cv::Mat4b> &frames);
    
protected:
    
    cv::Vec2d m_range;

    drishti::core::Field<PositionAndTime> m_previousStack;
    drishti::core::Field<PositionAndTime> m_previousPosition;
    double m_velocityTreshold = 0.1; // m/s
    double m_stackSampleInterval = 1.0f; // seconds
    uint64_t m_frameCounter = 0;
    uint64_t m_stackCounter = 0;
    
    std::shared_ptr<ThreadPool<128>> m_threads;
};

#endif // QT_FACE_MONITOR_H
