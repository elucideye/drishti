/*!
  @file   test-acf.cpp
  @author David Hirvonen
  @brief  Google test for the GPU ACF code.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/testlib/drishti_test_utils.h"

#if DRISHTI_BUILD_QT
#  include <QApplication>
#endif

#include <gtest/gtest.h>

extern const char* imageFilename;
extern const char* truthFilename;
extern const char* modelFilename;
extern const char* outputDirectory;

int drishti_main(int argc, char** argv)
{
#if DRISHTI_BUILD_QT && DRISHTI_HCI_DO_GPU
    QApplication app(argc, argv);
#endif
    
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc == 5);
    drishti::testlib::hasFiles(argv, {1,2,3});
    
    imageFilename = argv[1];
    truthFilename = argv[2];
    modelFilename = argv[3];
    outputDirectory = argv[4];
    
    return RUN_ALL_TESTS();
}
