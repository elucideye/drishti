/*!
  @file   Logger.h
  @author Ruslan Baratov
  @author David Hirvonen
  @brief  Declaration of logging class wrapper (spdlog)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_Logger_h__
#define __drishti_core_Logger_h__

#include <spdlog/spdlog.h>

#include "drishti/core/drishti_core.h"

#include <mutex>

DRISHTI_CORE_NAMESPACE_BEGIN

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

// Enable only one of these:
#define DRISHTI_DO_VERBOSE_LOGGING 0
#define DRISHTI_DO_COMPACT_LOGGING 0
#define DRISHTI_DO_NO_LOGGING 1

#if DRISHTI_DO_VERBOSE_LOGGING
// Verbose:
#define DRISHTI_STREAM_LOG_FUNC(0,1,ptr) do                      \
{                                                                \
    if(ptr)                                                      \
    {                                                            \
        ptr->info() << __PRETTY_FUNCTION__ << "::" << __func__;  \
    }                                                            \
} while (0)
#endif

#if DRISHTI_DO_COMPACT_LOGGING
// Minimal:
#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr) do   \
{                                                            \
   if(ptr)                                                   \
   {                                                         \
       if(drishti::core::Logger::count() < 999999)           \
       {                                                     \
           ptr->info() << FILE_ID << ":" << CHECKPOINT;      \
       }                                                     \
   }                                                         \
} while (0)
#endif

#if DRISHTI_DO_NO_LOGGING
// Disable:
#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr)
#endif

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_Logger_h__
