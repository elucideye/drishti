/*! -*-c++-*-
  @file   ThreadPool.h
  @author David Hirvonen
  @brief  Declaration of static ThreadPool storage

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_ThreadPool_h__
#define __drishti_core_ThreadPool_h__ 1

#include "drishti/core/drishti_core.h"
#include "thread_pool/thread_pool.hpp"

DRISHTI_CORE_NAMESPACE_BEGIN

class ThreadPoolSource
{
public:
    using FixedThreadPool = tp::ThreadPool<>;
    static FixedThreadPool* getInstance();

private:
    ThreadPoolSource() {}

public:
    ThreadPoolSource(ThreadPoolSource const&) = delete;
    void operator=(ThreadPoolSource const&) = delete;
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_ThreadPool_h__
