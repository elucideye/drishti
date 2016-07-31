/*!
  @file   drishti_master.hpp
  @author David Hirvonen (dhirvonen elucideye com)
  @brief  Minimal top level file to satisfy CMake + Xcode >= 1 source
  file when using CMake object libraries.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef DRISHTI_SDK_MASTER_H
#define DRISHTI_SDK_MASTER_H 1

#define DRISHTI_MASTER_BEGIN namespace master {
#define DRISHTI_MASTER_END }

DRISHTI_MASTER_BEGIN

// Some cmake projects require >= 1 non empty cpp file per library
struct VersionInfo
{
    VersionInfo();
};

DRISHTI_MASTER_END

#endif // DRISHTI_SDK_MASTER_H
