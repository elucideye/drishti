/*! -*-c++-*-
  @file   Logger.cpp
  @author Ruslan Baratov
  @author David Hirvonen
  @brief  Declaration of logging class wrapper (spdlog)

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/Logger.h"
#include <spdlog/spdlog.h>

// clang-format off
#if defined(__ANDROID__)
#  include <spdlog/sinks/android_sink.h>
#endif
// clang-format on

DRISHTI_CORE_NAMESPACE_BEGIN

std::mutex Logger::m_mutex;
int Logger::m_count = 0;

int Logger::count()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_count;
}

int Logger::increment()
{
    std::unique_lock<std::mutex> lock(m_mutex);
    return m_count++;
}

std::shared_ptr<spdlog::logger> Logger::create(const char* name)
{
    std::unique_lock<std::mutex> lock(m_mutex);

    {
        // Check for existing logger
        auto logger = get(name);
        if (logger)
        {
            return logger;
        }
    }

    std::vector<spdlog::sink_ptr> sinks;
    sinks.push_back(std::make_shared<spdlog::sinks::stdout_sink_mt>());
#if defined(__ANDROID__)
    sinks.push_back(std::make_shared<spdlog::sinks::android_sink>());
#endif
    auto logger = std::make_shared<spdlog::logger>(name, begin(sinks), end(sinks));
    spdlog::register_logger(logger);
    spdlog::set_pattern("[%H:%M:%S.%e | thread:%t | %n | %l]: %v");
    return logger;
}

std::shared_ptr<spdlog::logger> Logger::get(const char* name)
{
    return spdlog::get(name);
}

void Logger::drop(const char* name)
{
    spdlog::drop(name);
}

DRISHTI_CORE_NAMESPACE_END
