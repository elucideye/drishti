/*!
  @file   test-drishti-core.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

#include <iostream>
#include <stdlib.h>

#include <vector>
#include <string>

int test_main(int argc, char** argv)
{
    std::cout << "DRISHTI_TEST:" <<  getenv("HOME") << std::endl;
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}

#include "drishti/testing/main.cpp"
