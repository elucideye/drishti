/*!
  @file   drishti_serialize.h
  @author David Hirvonen
  @brief  Declaration of serialization utility functions.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_serialize_h__
#define __drishti_core_drishti_serialize_h__

inline bool isArchiveSupported(const std::string &filename)
{
#if DRISHTI_SERIALIZE_WITH_BOOST
    if(filename.find(".pba.z") != std::string::npos) return true;
#endif
    
#if DRISHTI_SERIALIZE_WITH_BOOST && DRISHTI_USE_TEXT_ARCHIVES
    if(filename.find(".txt") != std::string::npos) return true;
#endif
    
#if DRISHTI_SERIALIZE_WITH_CEREAL
    if(filename.find(".cpb") != std::string::npos) return true;
#endif
    
    return false;
}

#endif
