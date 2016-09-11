/*!
  @file   string_utils.h
  @author David Hirvonen
  @brief  Declaration of string manipulation routines.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef CORE_STRING_UTILS_H
#define CORE_STRING_UTILS_H

#include "drishti/core/drishti_core.h"
#include <sstream>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <iterator>
#include <memory>
#include <iomanip>

DRISHTI_CORE_BEGIN

void tokenize(const std::string &input, std::vector<std::string> &tokens);
std::string basename(const std::string &name, const std::string &ext=".");
bool replace(std::string& str, const std::string& from, const std::string& to);

DRISHTI_CORE_END

#endif /* CORE_STRING_UTILS_H_ */
