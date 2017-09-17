/*! -*-c++-*-
  @file   fddb/FDDB.h
  @author David Hirvonen
  @brief  FDDB dataset parsing.

  \copyright Copyright 2015-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_fddb_FDDB_h__
#define __drishti_fddb_FDDB_h__

#include <fstream>

// Test boost::spirit parsing:
//#define BOOST_SPIRIT_DEBUG
#include <boost/filesystem.hpp>
#include <boost/fusion/adapted/std_pair.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/repository/include/qi_seek.hpp>

namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

namespace FDDB
{
struct record
{
    typedef std::vector<double> Ellipse;
    typedef std::pair<Ellipse, bool> EllipseEntry;

    std::string filename;
    std::vector<EllipseEntry> ellipses;
    friend std::ostream& operator<<(const std::ostream& os, const record& r);
};

std::ostream& operator<<(std::ostream& os, const record& r);
}

std::vector<FDDB::record> parseFDDB(const std::string& filename);

#endif // FDDB_H
