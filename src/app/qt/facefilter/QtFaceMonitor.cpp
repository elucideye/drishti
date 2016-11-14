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

QtFaceMonitor::QtFaceMonitor(const cv::Vec2d &range)
: m_range(range)
, m_frameCounter(0)
, m_stackCounter(0)
{

}

bool QtFaceMonitor::isValid(const cv::Point3f &position)
{
    // Here we can apply various heuristics 
    m_frameCounter++;
    std::cout << position << std::endl;
    return !(m_frameCounter % 100);
}

void QtFaceMonitor::grab(std::vector<cv::Mat4b> &frames)
{
    m_stackCounter++;

    for(int i = 0; i < frames.size(); i++)
    {
        std::stringstream ss;
        ss << "/tmp/frame_" << m_stackCounter << "_"  << i << ".png";
        std::cout << "Logging: " << ss.str() << std::endl;
        cv::imwrite(ss.str(), frames[i]);
    }
}
