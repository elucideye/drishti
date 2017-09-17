/*! -*-c++-*-
  @file   ThreadPool.cpp
  @author David Hirvonen
  @brief  Implementation of static ThreadPool storage

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/ThreadPool.h"

DRISHTI_CORE_NAMESPACE_BEGIN

ThreadPoolSource::FixedThreadPool* ThreadPoolSource::getInstance()
{
    static FixedThreadPool instance;
    return &instance;
}

DRISHTI_CORE_NAMESPACE_END
