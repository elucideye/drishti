/*! -*-c++-*-
  @file  make_unique.h
  @brief Simple macro to add make_unique functionality currently lacking in C++11

  \copyright Copyright 2017-2018 Elucideye, Inc. All rights reserved. [All modifications]
  \license{This project is released under the 3 Clause BSD License.}

  Lineage: 

  http://clean-cpp.org/underprivileged-unique-pointers-make_unique/#more-54

*/

#ifndef __facefilter_make_unique_h__
#define __facefilter_make_unique_h__ 1

#include <memory>
#include <facefilter/facefilter.h>

BEGIN_FACEFILTER_NAMESPACE

template <typename Value, typename... Arguments>
std::unique_ptr<Value> make_unique(Arguments&&... arguments_for_constructor)
{
    return std::unique_ptr<Value>(new Value(std::forward<Arguments>(arguments_for_constructor)...));
}

END_FACEFILTER_NAMESPACE

#endif // __facefilter_make_unique_h__ 1
