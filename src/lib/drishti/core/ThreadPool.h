/*!
  @file   ThreadPool.h
  @author David Hirvonen
  @brief  Declaration of static ThreadPool storage

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_CORE_THREAD_POOL_H 
#define DRISHTI_CORE_THREAD_POOL_H 1

#include "drishti/core/drishti_core.h"
#include "thread_pool/thread_pool.hpp"

DRISHTI_CORE_NAMESPACE_BEGIN

// The static ThreadPool access works around a linker problem with thread_local storage 
// on iOS platforms which shows up when a class containing thread_local storage is dynamically
// allocated within a shared library (dynamic framework)

class ThreadPoolSource
{
public:
    using FixedThreadPool = ThreadPool<128>;
    static FixedThreadPool * getInstance();
private:
    ThreadPoolSource() {}
public:
    ThreadPoolSource(ThreadPoolSource const &)  = delete;
    void operator=(ThreadPoolSource const &)    = delete;
};

DRISHTI_CORE_NAMESPACE_END

#endif // DRISHTI_CORE_THREAD_POOL_H 
