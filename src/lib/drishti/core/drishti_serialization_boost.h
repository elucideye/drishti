/*!
  @file   drishti_serialization_boost.h
  @author David Hirvonen
  @brief  Common include for boost serialization headers.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_serialization_boost_h__
#define __drishti_core_drishti_serialization_boost_h__

#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>
#include <boost/serialization/list.hpp>
#include <boost/serialization/string.hpp>
#include <boost/serialization/version.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/utility.hpp>
#include <boost/iostreams/copy.hpp>
#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/iostreams/filter/gzip.hpp>
#include <boost/exception/exception.hpp>
#include <boost/exception/all.hpp>

#if !DRISHTI_BUILD_MIN_SIZE
#  include "boost-pba/portable_binary_oarchive.hpp"
#endif
#include "boost-pba/portable_binary_iarchive.hpp"

#endif
