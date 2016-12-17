/*!
  @file   timing.h
  @author Shervin Emami
  @brief  Common timer macros

  see: http://shervinemami.info/timingTests.html

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_timing_h__
#define __drishti_core_timing_h__

#include <opencv2/core/core.hpp>
#include <string>
#include <iostream>

#define DECLARE_TIMING(s)  int64 timeStart_##s; double timeDiff_##s; double timeTally_##s = 0; int countTally_##s = 0
#define START_TIMING(s)    timeStart_##s = cv::getTickCount()
#define STOP_TIMING(s) 	   timeDiff_##s = (double)(cv::getTickCount() - timeStart_##s); timeTally_##s += timeDiff_##s; countTally_##s++
#define GET_TIMING(s) 	   (double)(timeDiff_##s / (cv::getTickFrequency()))
#define GET_AVERAGE_TIMING(s)   (double)(countTally_##s ? timeTally_##s/ ((double)countTally_##s * cv::getTickFrequency()) : 0)
#define GET_TIME_TALLY(s)    (double)(timeTally_##s / (cv::getTickFrequency()))
#define CLEAR_AVERAGE_TIMING(s) timeTally_##s = 0; countTally_##s = 0
#define CLEAR_CURRENT_TIME(s) timeDiff_##s = 0
#define DRISHTI_PRINT(fmt, ...) do { fprintf(stderr, fmt, __VA_ARGS__); } while (0)

class ScopeTimer
{
public:
    ScopeTimer(const std::string &name, bool verbose=true) : m_verbose(verbose), name(name)
    {
        if(m_verbose)
        {
            START_TIMING(_timer_);
        }
    }
    ~ScopeTimer()
    {
        if(m_verbose)
        {
            STOP_TIMING(_timer_);
            std::cout << name << ":" << GET_TIMING(_timer_) << std::endl;
        }
    }
    bool m_verbose = false;
    std::string name;
    DECLARE_TIMING(_timer_);
};


#endif
