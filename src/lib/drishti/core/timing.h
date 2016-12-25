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

template <typename TimeLogger>
struct ScopeTimeLogger
{
    using HighResolutionClock = std::chrono::high_resolution_clock;
    using TimePoint = HighResolutionClock::time_point;
    
    ScopeTimeLogger(TimeLogger &logger) : m_logger(logger)
    {
        m_tic = HighResolutionClock::now();
    }
    ~ScopeTimeLogger()
    {
        auto now = HighResolutionClock::now();
        m_logger(timeDifference(now, m_tic));
    }
    static double timeDifference(const TimePoint &a, const TimePoint &b)
    {
        return std::chrono::duration_cast<std::chrono::duration<double>>(a - b).count();
    }
    
    TimeLogger m_logger;
    TimePoint m_tic;
};

#endif // __drishti_core_timing_h__
