/*! -*-c++-*-
  @file   Semaphore.h
  @author David Hirvonen
  @brief  Simple Semaphore from C++11 primitives.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_Semaphore_h__
#define __drishti_core_Semaphore_h__ 1

#include "drishti/core/drishti_core.h"
#include "thread_pool/thread_pool.hpp"

DRISHTI_CORE_NAMESPACE_BEGIN

class Semaphore
{
public:
    Semaphore(std::size_t n = 0)
        : n(n)
    {
    }
    void wait()
    {
        std::unique_lock<std::mutex> lock(mutex);
        condition.wait(lock, [&]() { return n > 0; });
        --n;
    }
    void signal()
    {
        std::unique_lock<std::mutex> lock(mutex);
        ++n;
        condition.notify_one();
    }

protected:
    std::mutex mutex;
    std::condition_variable condition;
    std::size_t n = 0;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_Semaphore_h__
