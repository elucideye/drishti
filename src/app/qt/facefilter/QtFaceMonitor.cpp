/*!
  @file   QtFaceMonitor.cpp
  @author David Hirvonen
  @brief Qt sample facemonitor 

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "QtFaceMonitor.h"

#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <vector>
#include <iostream>


double QtFaceMonitor::PositionAndTime::estimateVelocity(const PositionAndTime &current)
{
    double translation = cv::norm(current.position - position);
    double interval = current.time - time;
    return translation / interval;
}

QtFaceMonitor::QtFaceMonitor(const cv::Vec2d &range, std::shared_ptr<ThreadPool<128>> &threads)
: m_range(range)
, m_frameCounter(0)
, m_stackCounter(0)
, m_threads(threads)
{
    
}

bool QtFaceMonitor::isValid(const cv::Point3f &position, double timeStamp)
{
    bool okay = false;
    
    // Here we can apply various heuristics 
    m_frameCounter++;
    PositionAndTime current(position, timeStamp);
    
    if(m_previousPosition.has)
    {
        const double velocity = m_previousPosition->estimateVelocity(current);
        std::cout << "Velocity: " << velocity << " " << position << std::endl;
        
        if((m_range[0] < position.z) && (position.z < m_range[1]))
        {
            
            if(velocity < m_velocityTreshold)
            {
                if((timeStamp - m_previousStack->time) > m_stackSampleInterval)
                {
                    okay = true;
                    m_previousStack = current;
                }
            }
        }
    }

    m_previousPosition = current;

    return okay;
}

void QtFaceMonitor::grab(std::vector<cv::Mat4b> &frames)
{
    auto counter = m_stackCounter++;
    std::function<void()> logger = [frames, counter, this]()
    {
        for(int i = 0; i < frames.size(); i++)
        {
            std::stringstream ss;
            ss << "/tmp/frame_" << counter << "_"  << i << ".png";
            std::cout << "Logging: " << ss.str() << std::endl;
            cv::imwrite(ss.str(), frames[i]);
        }
    };
    
    if(m_threads)
    {
        m_threads->process(logger);
    }
    else
    {
        logger();
    }
}
