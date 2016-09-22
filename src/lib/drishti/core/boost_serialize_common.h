/*!
  @file   boost_serialize_common.h
  @author David Hirvonen
  @brief  Private header to support easy reuse of common boost serialization includes.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef drishtisdk_boost_serialize_common_h
#define drishtisdk_boost_serialize_common_h

#include <opencv2/core.hpp>
#include <fstream>


// Boost serialization files:
#if DRISHTI_USE_TEXT_ARCHIVES
#  include <boost/archive/text_iarchive.hpp>
#  include <boost/archive/text_oarchive.hpp>
#endif

#include <boost/serialization/vector.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/export.hpp>

#include <boost/iostreams/filtering_stream.hpp>
#include <boost/iostreams/filtering_streambuf.hpp>
#include <boost/iostreams/filter/zlib.hpp>
#include <boost/serialization/shared_ptr.hpp>
#include <boost/serialization/vector.hpp>
#include <boost/serialization/export.hpp>

#include <boost/serialization/level.hpp>
#include <boost/serialization/tracking.hpp>

#include "portable_binary_iarchive.hpp"
#include "portable_binary_oarchive.hpp"

// ############################################################
// ######################## PBA ###############################
// ############################################################

template <typename T>
void load_pba_z(std::istream &is, T &object)
{
    //CV_Assert(is);
    boost::iostreams::filtering_stream<boost::iostreams::input> buffer;
    buffer.push(boost::iostreams::zlib_decompressor());
    buffer.push(is);
    portable_binary_iarchive ia(buffer);
    ia >> object;
}

template <typename T>
void load_pba_z(const std::string &filename, T &object)
{
    std::ifstream ifs(filename, std::ios::binary);
    CV_Assert(ifs);
    load_pba_z(ifs, object);
}

template <typename T>
void save_pba_z(std::ostream &os, T &object)
{
#if !DRISHTI_BUILD_MIN_SIZE
    boost::iostreams::filtering_stream<boost::iostreams::output> buffer;
    buffer.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_compression));
    buffer.push(os);
    portable_binary_oarchive oa(buffer);
    oa << object;
#endif
}

template <typename T>
void save_pba_z(const std::string &filename, T &object)
{
#if !DRISHTI_BUILD_MIN_SIZE    
    std::ofstream ofs(filename, std::ios::binary);
    CV_Assert(ofs);
    save_pba_z(ofs, object);
#endif
}

// ############################################################
// ######################## TEXT ##############################
// ############################################################

#if DRISHTI_USE_TEXT_ARCHIVES

template <typename T>
void load_txt_z(std::istream &is, T &object)
{
    boost::archive::text_iarchive ia(is);
    ia >> object;
}

template <typename T>
void load_txt_z(const std::string &filename, T &object)
{
    std::ifstream ifs(filename, std::ios::binary);
    load_txt_z(ifs, object);
}

template <typename T>
void save_txt_z(std::ostream &os, T &object)
{
    boost::archive::text_oarchive oa(os);
    oa << object;
}

template <typename T>
void save_txt_z(const std::string &filename, T &object)
{
    std::ofstream ofs(filename, std::ios::binary);
    save_txt_z(ofs, object);
}

#endif // DRISHTI_USE_TEXT_ARCHIVES

#endif
