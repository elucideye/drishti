/*! -*-c++-*-
  @file   string_utils.cpp
  @author David Hirvonen
  @brief  Implementation of string manipulation routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <stdio.h>

#include "drishti/core/string_utils.h"
#include "drishti/core/Line.h" // for csv_reader
#include <locale>

DRISHTI_CORE_NAMESPACE_BEGIN

void tokenize(const std::string& input, std::vector<std::string>& tokens)
{
    std::stringstream iss(input);
    iss.imbue(std::locale(std::locale(), new csv_reader()));
    std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));
}

std::string basename(const std::string& name, const std::string& ext)
{
    size_t pos = name.rfind("/");

    if (pos != std::string::npos)
    {
        pos += 1;
    }
    else
    {
        pos = 0;
    }

    std::string base = name.substr(pos);
    return base.substr(0, std::min(base.size(), base.rfind(ext)));
};

bool replace(std::string& str, const std::string& from, const std::string& to)
{
    bool status = false;
    size_t pos = str.find(from);
    if (pos != std::string::npos)
    {
        status = true;
        str.replace(pos, from.length(), to);
    }
    return status;
}

DRISHTI_CORE_NAMESPACE_END
