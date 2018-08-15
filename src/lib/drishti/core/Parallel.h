/*! -*-c++-*-
  @file   Parallel.h
  @author David Hirvonen
  @brief  Declaration of parallel_for abstraction classes

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_Parallel_h__
#define __drishti_core_Parallel_h__

#include "drishti/core/drishti_core.h"
#include <opencv2/core/core.hpp>
#include <functional>

DRISHTI_CORE_NAMESPACE_BEGIN

struct ParallelHeterogeneousLambda : public cv::ParallelLoopBody
{
public:
    ParallelHeterogeneousLambda(std::vector<std::function<void()>>& functions)
        : m_functions(functions)
    {
    }
    void operator()(const cv::Range& range) const override
    {
        for (int i = range.start; i < range.end; i++)
        {
            m_functions[i]();
        }
    }

    ~ParallelHeterogeneousLambda() override = default;

    ParallelHeterogeneousLambda(const ParallelHeterogeneousLambda&) = delete;
    ParallelHeterogeneousLambda(ParallelHeterogeneousLambda&&) = delete;
    ParallelHeterogeneousLambda& operator=(const ParallelHeterogeneousLambda&) = delete;
    ParallelHeterogeneousLambda& operator=(ParallelHeterogeneousLambda&&) = delete;

protected:
    std::vector<std::function<void()>> m_functions;
};

struct ParallelHomogeneousLambda : public cv::ParallelLoopBody
{
public:
    ParallelHomogeneousLambda(std::function<void(int)>& function)
        : m_function(function)
    {
    }

    template <class Callable>
    ParallelHomogeneousLambda(Callable&& func)
        : m_function(std::forward<Callable>(func))
    {
    }

    ParallelHomogeneousLambda(ParallelHomogeneousLambda&& other) noexcept
        : m_function(std::move(other.m_function))
    {
        other.m_function = nullptr;
    }

    void operator()(const cv::Range& range) const override
    {
        for (int i = range.start; i < range.end; i++)
        {
            m_function(i);
        }
    }

    ParallelHomogeneousLambda(const ParallelHomogeneousLambda&) = delete;
    ParallelHomogeneousLambda operator=(const ParallelHomogeneousLambda&) = delete;
    ParallelHomogeneousLambda operator=(ParallelHomogeneousLambda&&) = delete;

    ~ParallelHomogeneousLambda() override = default;

protected:
    std::function<void(int)> m_function;
};

struct ParallelLambdaRange : public cv::ParallelLoopBody
{
public:
    ParallelLambdaRange(std::function<void(const cv::Range& r)>& function)
        : m_function(function)
    {
    }

    template <class Callable>
    ParallelLambdaRange(Callable&& func)
        : m_function(std::forward<Callable>(func))
    {
    }

    ParallelLambdaRange(ParallelLambdaRange&& other) noexcept
        : m_function(std::move(other.m_function))
    {
        other.m_function = nullptr;
    }

    void operator()(const cv::Range& range) const override
    {
        m_function(range);
    }

    ParallelLambdaRange(const ParallelLambdaRange&) = delete;
    ParallelLambdaRange& operator=(const ParallelLambdaRange&) = delete;
    ParallelLambdaRange& operator=(ParallelLambdaRange&&) = delete;

    ~ParallelLambdaRange() override = default;

protected:
    std::function<void(const cv::Range& r)> m_function;
};

DRISHTI_CORE_NAMESPACE_END

#endif
