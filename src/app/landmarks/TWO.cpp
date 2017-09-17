/*! -*-c++-*-
 @file   TWO.cpp
 @author David Hirvonen
 @brief  High level routines for parsing TWO data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

//#define BOOST_SPIRIT_DEBUG

#include "landmarks/TWO.h"

#include "drishti/core/drishti_core.h"

#include <opencv2/core.hpp>

#include <fstream>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>

namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

DRISHTI_BEGIN_NAMESPACE(TWO)

struct record
{
    std::string filename;
    std::vector<cv::Point2f> points;
};

struct Table
{
    std::vector<record> lines;
};

DRISHTI_END_NAMESPACE(TWO)

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    TWO::Table,
    (std::vector<TWO::record>, lines)
)
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    TWO::record,
    (std::string, filename)
    (std::vector<cv::Point2f>, points)
)
// clang-format on

DRISHTI_BEGIN_NAMESPACE(TWO)

template <typename Iterator, typename Skipper = qi::blank_type>
struct two_parser : qi::grammar<Iterator, TWO::Table(), Skipper>
{
    two_parser()
        : two_parser::base_type(start)
    {
        point = (qi::float_ >> qi::float_);
        points = qi::repeat(68)[point];

        //filename = qi::char_
        filename = qi::lexeme[*(qi::char_("a-zA-Z0-9_/.:"))];

        line = (filename >> qi::eol >> points >> (qi::eol | qi::eoi));
        start = +line >> qi::eoi;
        BOOST_SPIRIT_DEBUG_NODES((start)(line)(filename)(point)(points))
    }

    qi::rule<Iterator, cv::Point2f(), Skipper> point;
    qi::rule<Iterator, std::vector<cv::Point2f>(), Skipper> points;
    qi::rule<Iterator, std::string(), Skipper> filename;
    qi::rule<Iterator, TWO::record(), Skipper> line;
    qi::rule<Iterator, TWO::Table(), Skipper> start;
};

DRISHTI_END_NAMESPACE(TWO)

FACE::Table parseTWO(const std::string& filename)
{
    FACE::Table table;

    table.browR = { 17, 18, 19, 20, 21 };
    table.browL = { 22, 23, 24, 25, 26 };
    table.eyeR = { 36, 37, 38, 39, 40, 41 };
    table.eyeL = { 42, 43, 44, 45, 46, 47 };
    table.nose = { 27, 28, 29, 30, 31, 32, 33, 34, 35 };
    table.mouth = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 };
    table.mouthR = { 48 };
    table.mouthL = { 54 };
    table.brow = {};

    // open file, disable skipping of whitespace
    std::ifstream ifs(filename);

    if (ifs)
    {
        ifs.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator f(ifs), l;

        TWO::two_parser<decltype(f)> parser;

        TWO::Table result;
        bool success = qi::phrase_parse(f, l, parser, qi::blank, result);

        table.lines.resize(result.lines.size());
        for (int i = 0; i < result.lines.size(); i++)
        {
            table.lines[i].filename = result.lines[i].filename;
            table.lines[i].points = result.lines[i].points;
        }
    }

    return table;
}
