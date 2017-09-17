/*! -*-c++-*-
  @file   drishti_test_utils.h
  @author David Hirvonen
  @brief  Test utilities.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_testlib_drishti_test_utils_h__
#define __drishti_testlib_drishti_test_utils_h__

#include "drishti/testlib/drishti_testlib.h"

#include <vector>
#include <fstream>
#include <cassert>

DRISHTI_TESTLIB_NAMESPACE_BEGIN

inline bool hasFile(const std::string& filename)
{
    std::ifstream ifs(filename, std::ios::binary | std::ios::in);
    return ifs.good();
}

inline bool hasFiles(char** argv, const std::vector<int>& index)
{
    for (const auto& i : index)
    {
        assert(hasFile(argv[i]));
    }
    return true;
}

DRISHTI_TESTLIB_NAMESPACE_END

#endif
