/*!
  @file   test-drishti-eye.cpp
  @author David Hirvonen
  @brief  Google test for public drishti API.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include <gtest/gtest.h>

extern const char* modelFilename;
extern const char* imageFilename;
extern const char* truthFilename;
extern const char* outputDirectory;
extern bool isTextArchive;

#include <fstream>

static bool hasFile(const std::string &filename)
{
    std::ifstream ifs(filename);
    return ifs.good();
}

int drishti_main(int argc, char** argv)
{
    ::testing::InitGoogleTest(&argc, argv);
    assert(argc >= 4);
    modelFilename = argv[1];
    imageFilename = argv[2];
    truthFilename = argv[3];
    outputDirectory = argv[4];
    isTextArchive = (argc > 5) ? (std::atoi(argv[5]) > 0) : false;
    
    assert(hasFile(modelFilename));
    assert(hasFile(imageFilename));
    assert(hasFile(truthFilename));

    return RUN_ALL_TESTS();
}
