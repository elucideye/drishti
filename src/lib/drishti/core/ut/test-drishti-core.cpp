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

int main(int argc, char** argv)
{
    std::string home = getenv("HOME");
    std::cout << "DRISHTI_TEST:" <<  argv[0] << " = " << home << std::endl;
    
    ::testing::InitGoogleTest(&argc, argv);
    return RUN_ALL_TESTS();
}
