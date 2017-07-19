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
// clang-format off
#define DRISHTI_STREAM_LOG_FUNC(0,1,ptr) do                      \
{                                                                \
    if(ptr)                                                      \
    {                                                            \
        ptr->info("{} :: {}", __PRETTY_FUNCTION__, __func__);    \
    }                                                            \
} while (0)
// clang-format on
#endif

#if DRISHTI_DO_COMPACT_LOGGING
// Minimal:
// clang-format off
#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr) do   \
{                                                            \
   if(ptr)                                                   \
   {                                                         \
       if(drishti::core::Logger::count() < 999999)           \
       {                                                     \
           ptr->info("{} : {}", FILE_ID, CHECKPOINT);        \
       }                                                     \
   }                                                         \
} while (0)
// clang-format on
#endif

#if DRISHTI_DO_NO_LOGGING
// Disable:
// clang-format off
#define DRISHTI_STREAM_LOG_FUNC(FILE_ID,CHECKPOINT,ptr)
// clang-format on
#endif

/*
 * Logging macros: pretty printing, method names, etc
 */

// Note: __func__ = static const char __func__[] = "function-name";

// http://stackoverflow.com/a/15775519
inline std::string methodName(const std::string& prettyFunction)
{
    std::size_t colons = prettyFunction.find("::");
    std::size_t begin = prettyFunction.substr(0, colons).rfind(" ") + 1;
    std::size_t end = prettyFunction.rfind("(") - begin;

    return prettyFunction.substr(begin, end) + "()";
}

// http://stackoverflow.com/a/15775519
inline std::string className(const std::string& prettyFunction)
{
    std::size_t colons = prettyFunction.find("::");
    if (colons == std::string::npos)
    {
        return "::";
    }
    std::size_t begin = prettyFunction.substr(0, colons).rfind(" ") + 1;
    std::size_t end = colons - begin;

    return prettyFunction.substr(begin, end);
}

// clang-format off
#if _MSC_VER
#  define __METHOD_NAME__ drishti::core::methodName(__FUNCSIG__)
#  define __CLASS_NAME__ drishti::core::className(__FUNCSIG__)
#else
#  define __METHOD_NAME__ drishti::core::methodName(__PRETTY_FUNCTION__)
#  define __CLASS_NAME__ drishti::core::className(__PRETTY_FUNCTION__)
#endif
// clang-format on

// clang-format off
#define DRISHTI_TO_STR_(x) #x
#define DRISHTI_TO_STR(x) DRISHTI_TO_STR_(x)
#define DRISHTI_LOCATION_FULL std::string(__PRETTY_FUNCTION__)
#define DRISHTI_LOCATION_SIMPLE __CLASS_NAME__ + "::" + __METHOD_NAME__
// clang-format on

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_Logger_h__
