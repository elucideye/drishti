/**
  @file   drishti_sdk.cpp
  @author David Hirvonen
  @brief  Top level API eye model declaration.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file contains utilities for reporting version information, and is
  solely maintained for backwards compatibility.
*/

#include "drishti/drishti_sdk.hpp"

_DRISHTI_SDK_BEGIN

std::string getVersion()
{
    std::string version = DRISHTI_VERSION;
    return version;
}

_DRISHTI_SDK_END
