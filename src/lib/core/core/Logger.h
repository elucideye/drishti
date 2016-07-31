/*!
  @file   Logger.h
  @author Ruslan Baratov
  @author David Hirvonen
  @brief  Declaration of logging class wrapper (spdlog)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core__Logger__
#define __drishti_core__Logger__

#include <spdlog/spdlog.h>

#include "core/drishti_core.h"

#include <mutex>

DRISHTI_CORE_BEGIN

class Logger
{
public:
    using Pointer = std::shared_ptr<spdlog::logger>;
    static Pointer create(const char* name);
    static Pointer get(const char* name);
    static void drop(const char* name);
    static int count();
    static int increment();

protected:
    static std::mutex m_mutex;
    static int m_count;
};

// Verbose:
//#define DRISHTI_STREAM_LOG_FUNC(0,1,ptr) do \
//{                                           \
//    if(ptr)                                 \
//    {                                                         \
//        ptr->info() << __PRETTY_FUNCTION__ << "::" << __func__;   \
//    }                                                             \
//} while (0)

// Minimal:
//#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr) do    \
//{                                                             \
//    if(ptr)                                                   \
//    {                                                         \
//        if(drishti::core::Logger::count() < 4)                \
//        {                                                     \
//            ptr->info() << FILE_ID << ":" << CHECKPOINT;      \
//        }                                                     \
//    }                                                         \
//} while (0)

// Disable:
#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr)

DRISHTI_CORE_END

#endif // __drishti_core__Logger__
