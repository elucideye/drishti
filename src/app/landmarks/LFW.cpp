/*! -*-c++-*-
 @file   LFW.cpp
 @author David Hirvonen
 @brief  High level routines for parsing LFW data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

//#define BOOST_SPIRIT_DEBUG

#include "landmarks/LFW.h"

#include <fstream>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>

namespace ph = boost::phoenix;
namespace qi = boost::spirit::qi;
namespace ascii = boost::spirit::ascii;

DRISHTI_BEGIN_NAMESPACE(LFW)

struct record
{
    std::string filename;
    cv::Rect roi;
    std::vector<cv::Point2f> points;
};

struct Table
{
    std::vector<record> lines;
};

DRISHTI_END_NAMESPACE(LFW)

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    cv::Rect,
    (int, x)
    (int, y)
    (int, width)
    (int, height)
 )
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    LFW::Table,
    (std::vector<LFW::record>, lines)
)
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    LFW::record,
    (std::string, filename)
    (cv::Rect, roi)(std::vector<cv::Point2f>, points)
)
// clang-format on

DRISHTI_BEGIN_NAMESPACE(LFW)

template <typename Iterator, typename Skipper = qi::blank_type>
struct lfw_parser : qi::grammar<Iterator, LFW::Table(), Skipper>
{
    lfw_parser()
        : lfw_parser::base_type(start)
    {
        point = qi::float_ >> qi::float_;
        points = qi::repeat(5)[point];
        roi = qi::int_ >> qi::int_ >> qi::int_ >> qi::int_;
        filename = qi::lexeme[+(qi::char_ - qi::blank)];
        line = filename >> roi >> points >> qi::eol;
        start = +line >> qi::eoi;

        //BOOST_SPIRIT_DEBUG_NODES( (start)(line)(filename)(roi)(point)(points) )
    }

    qi::rule<Iterator, cv::Point2f(), Skipper> point;
    qi::rule<Iterator, std::vector<cv::Point2f>(), Skipper> points;
    qi::rule<Iterator, cv::Rect(), Skipper> roi;
    qi::rule<Iterator, std::string(), Skipper> filename;
    qi::rule<Iterator, LFW::record(), Skipper> line;
    qi::rule<Iterator, LFW::Table(), Skipper> start;
};

DRISHTI_END_NAMESPACE(LFW)

FACE::Table parseLFW(const std::string& filename)
{
    FACE::Table table;
    table.eyeR = { 0 };
    table.eyeL = { 1 };
    table.nose = { 2 };
    table.mouth = { 3, 4 };
    table.mouthR = { 3 };
    table.mouthL = { 4 };

    // open file, disable skipping of whitespace
    std::ifstream file(filename);

    if (file)
    {
        file.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator begin(file), end;
        LFW::lfw_parser<decltype(begin)> parser;

        LFW::Table result;
        bool success = qi::phrase_parse(begin, end, parser, qi::blank, result);

        table.lines.resize(result.lines.size());
        for (int i = 0; i < result.lines.size(); i++)
        {
            table.lines[i].roi = result.lines[i].roi;
            table.lines[i].filename = result.lines[i].filename;
            table.lines[i].points = result.lines[i].points;
        }
    }

    return table;
}
