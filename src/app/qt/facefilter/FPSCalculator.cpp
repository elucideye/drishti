/*! -*-c++-*-
  @file   finder/FPSCalculator.cpp
  @author Ruslan Baratov
  @brief  Frame rate calculator for QT app.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FPSCalculator.hpp"

#include <cassert> // assert

FPSCalculator::FPSCalculator()
    : count_(0)
    , fps_(0)
    , alpha_(0.95f)
{
}

float FPSCalculator::fps()
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
    const float elapsed = std::chrono::duration_cast<std::chrono::duration<float>>(now - last_).count();
    const float fps = 1.f / elapsed;
    fps_ = (count_ > 0) ? ((fps_ * alpha_) + ((1.0 - alpha_) * fps)) : fps;
    last_ = now;
    ++count_;
    return fps_;
}
