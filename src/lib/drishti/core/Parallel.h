/*!
  @file   Parallel.h
  @author David Hirvonen
  @brief  Declaration of parallel_for abstraction classes

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_Parallel_h
#define drishtisdk_Parallel_h

#include "drishti/core/drishti_core.h"
#include <opencv2/core/core.hpp>

DRISHTI_CORE_BEGIN

struct ParallelHeterogeneousLambda : public cv::ParallelLoopBody
{
public:
    ParallelHeterogeneousLambda(std::vector<std::function<void()> > &functions) : m_functions(functions) {}
    void operator()(const cv::Range &range) const
    {
        for(int i = range.start; i < range.end; i++)
        {
            m_functions[i]();
        }
    }
    std::vector<std::function<void()>> m_functions;
};

struct ParallelHomogeneousLambda : public cv::ParallelLoopBody
{
public:
    ParallelHomogeneousLambda(std::function<void(int)> &function) : m_function(function) {}
    void operator()(const cv::Range &range) const
    {
        for(int i = range.start; i < range.end; i++)
        {
            m_function(i);
        }
    }
    std::function<void(int)> m_function;
};

struct ParallelLambdaRange : public cv::ParallelLoopBody
{
public:
    ParallelLambdaRange(std::function<void(const cv::Range &r)> &function)  : m_function(function) {}
    void operator()(const cv::Range &range) const
    {
        m_function(range);
    }
    std::function<void(const cv::Range &r)> m_function;
};

DRISHTI_CORE_END

#endif
