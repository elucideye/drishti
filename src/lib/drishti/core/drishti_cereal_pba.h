/*! -*-c++-*-
  @file   drishti_cereal_pba.h
  @author David Hirvonen
  @brief  Provides compatibility with boost::serialization Archive::is_loading::value

*/

#ifndef __drishti_core_drishti_cereal_pba_h__
#define __drishti_core_drishti_cereal_pba_h__

#include "drishti/core/drishti_core.h"
#include "drishti/core/make_unique.h"

// http://uscilab.github.io/cereal/serialization_archives.html
#include <cereal/cereal.hpp>
#include <cereal/archives/portable_binary.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/string.hpp>
#include <cereal/types/memory.hpp>

#include <cassert>
#include <fstream>

inline bool is_cpb(std::istream& is)
{
    bool ok = false;
    is.seekg(0, std::ios_base::beg);
    try
    {
        cereal::PortableBinaryInputArchive ia(is);
        ok = true;
    }
    catch (...)
    {
    }

    is.seekg(0, std::ios_base::beg);
    is.clear();
    return ok;
}

template <typename T>
void load_cpb(std::istream& is, T& object)
{
    cereal::PortableBinaryInputArchive ia(is);
    ia >> object;
}

template <typename T>
void load_cpb(const std::string& filename, T& object)
{
    std::ifstream ifs(filename, std::ios::binary);
    assert(ifs);
    load_cpb(ifs, object);
}

template <typename T>
std::unique_ptr<T> make_unique_cpb(std::istream& is)
{
    auto ptr = drishti::core::make_unique<T>();
    load_cpb(is, *ptr);
    return ptr;
}

template <typename T>
std::unique_ptr<T> make_unique_cpb(const std::string& filename)
{
    auto ptr = drishti::core::make_unique<T>();
    load_cpb(filename, *ptr);
    return ptr;
}

template <typename T>
void save_cpb(std::ostream& os, T& object)
{
    cereal::PortableBinaryOutputArchive oa(os);
    oa << object;
}

template <typename T>
void save_cpb(const std::string& filename, T& object)
{
    std::ofstream ofs(filename, std::ios::binary);
    assert(ofs);
    save_cpb(ofs, object);
}

#endif // __drishti_core_drishti_cereal_pba_h__
