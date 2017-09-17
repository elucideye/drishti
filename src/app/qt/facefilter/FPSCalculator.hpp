/*! -*-c++-*-
  @file   finder/FPSCalculator.hpp
  @author Ruslan Baratov
  @brief  Frame rate calculator for QT app.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef FPSCALCULATOR_HPP_
#define FPSCALCULATOR_HPP_

#include <chrono>

class FPSCalculator
{
public:
    using Clock = std::chrono::high_resolution_clock;
    using Duration = Clock::duration;
    using TimePoint = Clock::time_point;
    using MS = std::chrono::milliseconds;

    FPSCalculator();

    float fps();

private:
    TimePoint last_;
    int count_;
    float fps_;
    float alpha_ = 0.95f;
};

#endif // FPSCALCULATOR_HPP_
