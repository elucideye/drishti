/*! -*-c++-*-
  @file   drishti_master.hpp
  @author David Hirvonen
  @brief  Minimal top level file to satisfy CMake + Xcode >= 1 source
  file when using CMake object libraries.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_master_drishti_master_hpp__
#define __drishti_master_drishti_master_hpp__ 1

// clang-format off
#define DRISHTI_MASTER_BEGIN  namespace master {
#define DRISHTI_MASTER_END }
// clang-format on

DRISHTI_MASTER_BEGIN

// Some cmake projects require >= 1 non empty cpp file per library
struct VersionInfo
{
    VersionInfo();
};

DRISHTI_MASTER_END

#endif // __drishti_master_drishti_master_hpp__
