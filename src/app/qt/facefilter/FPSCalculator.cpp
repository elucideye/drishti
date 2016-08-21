/*!
  @file   finder/FPSCalculator.cpp
  @author Ruslan Baratov
  @brief  Frame rate calculator for QT app.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FPSCalculator.hpp"

#include <cassert> // assert

FPSCalculator::FPSCalculator(): count_(0), fps_(0)
{
}

int FPSCalculator::fps()
{
    if (last_ == TimePoint())
    {
        // initialize
        last_ = Clock::now();
        count_ = 0;
        fps_ = 0;
        return 0;
    }

    const TimePoint now = Clock::now();
    assert(last_ <= now);

    ++count_;

    const Duration WINDOW = std::chrono::duration_cast<Duration>(MS(200));

    const Duration period = now - last_;
    if (period < WINDOW)
    {
        // Emit old FPS value, keep collecting information
        return fps_;
    }

    auto period_ms = std::chrono::duration_cast<MS>(period).count();

    assert(period_ms != 0);

    fps_ = static_cast<int>(1000.0 * count_ / period_ms);

    last_ = now;
    count_ = 0;

    return fps_;
}
