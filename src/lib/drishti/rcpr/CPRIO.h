/*! -*-c++-*-
  @file   CPRIO.h
  @author David Hirvonen (C++ implementation (gradient boosting trees))
  @brief  Declaration of (de)serialization routines for Cascaded Pose Regression class.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

/*

 std::string sTest = R"(% poseGt version=2 m=1
 185.408 126.308 6.56514 6.36167 -0.132587
 )";

 void testRCPR()
 {
 std::cout << sTest << std::endl;
 drishti::rcpr::EllipseEntry ellipse;
 drishti::rcpr::record_parser<decltype(sTest.begin())> parser;
 bool success = qi::phrase_parse(sTest.begin(), sTest.end(), parser, qi::blank, ellipse);
 std::cout << int(success) << "\n" << ellipse << std::endl;
 }

 */

#ifndef __drishti_rcpr_CPRIO_h__
#define __drishti_rcpr_CPRIO_h__

#include "drishti/rcpr/drishti_rcpr.h"

#include <boost/filesystem.hpp>
#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>
#include <boost/spirit/repository/include/qi_seek.hpp>

#include <iostream>

namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

DRISHTI_RCPR_NAMESPACE_BEGIN

struct EllipseEntry
{
    int version;
    int parts;
    drishti::rcpr::Vector1d ellipse;
    friend std::ostream& operator<<(const std::ostream& os, const EllipseEntry& r);
};

std::ostream& operator<<(std::ostream& os, const EllipseEntry& r)
{
    for (auto& i : r.ellipse)
    {
        os << i << ' ';
    }

    return os;
}

DRISHTI_RCPR_NAMESPACE_END

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    drishti::rcpr::EllipseEntry,
    (int, version)
    (int, parts)
    (drishti::rcpr::Vector1d, ellipse)
)
// clang-format on

DRISHTI_RCPR_NAMESPACE_BEGIN

template <typename Iterator, typename Skipper = qi::blank_type>
struct record_parser : qi::grammar<Iterator, EllipseEntry(), Skipper>
{
    record_parser()
        : record_parser::base_type(start)
    {
        params = qi::repeat(5)[(qi::double_ % ' ')] >> (qi::eol | qi::eoi);
        start = qi::lexeme["%"] >> "poseGt" >> "version=" >> qi::int_ >> "m=" >> qi::int_ >> qi::lexeme[qi::eol] >> params;
        BOOST_SPIRIT_DEBUG_NODE(params);
        BOOST_SPIRIT_DEBUG_NODE(start);
    }
    qi::rule<Iterator, std::vector<double>(), Skipper> params;
    qi::rule<Iterator, rcpr::EllipseEntry(), Skipper> start;
};

DRISHTI_RCPR_NAMESPACE_END

#endif
