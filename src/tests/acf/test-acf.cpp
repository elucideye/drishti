/*!
  @file   test-acf.cpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Google test for the GPU ACF code.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/


#include <QApplication>
#include <gtest/gtest.h>

extern const char* imageFilename;
extern const char* truthFilename;

int main(int argc, char** argv)
{
    QApplication app(argc, argv);

    ::testing::InitGoogleTest(&argc, argv);
    assert(argc == 3);
    imageFilename = argv[1];
    truthFilename = argv[2];
    return RUN_ALL_TESTS();
}
