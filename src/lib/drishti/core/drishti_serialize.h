/*! -*-c++-*-
  @file   drishti_serialize.h
  @author David Hirvonen
  @brief  Declaration of serialization utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_serialize_h__
#define __drishti_core_drishti_serialize_h__

inline bool isArchiveSupported(const std::string& filename)
{
    if (filename.find(".cpb") != std::string::npos)
        return true;

    return false;
}

#endif // __drishti_core_drishti_serialize_h__
