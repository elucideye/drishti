/*! -*-c++-*-
  @file   drishti_core.h
  @author David Hirvonen
  @brief  Declaration of internal drishti core namespace.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_core_h__
#define __drishti_core_drishti_core_h__

// clang-format off
#define DRISHTI_BEGIN_NAMESPACE(X) namespace X {
#define DRISHTI_END_NAMESPACE(X) }
// clang-format on

// clang-format off
#define _DRISHTI_BEGIN namespace drishti {
#define _DRISHTI_END }
// clang-format on

// clang-format off
#define DRISHTI_CORE_NAMESPACE_BEGIN namespace drishti { namespace core {
#define DRISHTI_CORE_NAMESPACE_END } }
// clang-format on

#endif
