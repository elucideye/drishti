/*! -*-c++-*-
 @file   HELEN.cpp
 @author David Hirvonen
 @brief  High level routines for (de)serialization and manipulation of flattened HELEN data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "landmarks/HELEN.h"

#include <fstream>

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>

namespace qi = boost::spirit::qi;
namespace phx = boost::phoenix;

// HELEN/helen.csv
// 100032540_1,565.86,758.98,564.27,

DRISHTI_BEGIN_NAMESPACE(HELEN)

struct record
{
    std::string filename;
    std::vector<cv::Point2f> points;
};
struct Table
{
    std::vector<record> lines;
};

DRISHTI_END_NAMESPACE(HELEN)

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    HELEN::Table,
    (std::vector<HELEN::record>, lines)
)
// clang-format on

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    HELEN::record,
    (std::string, filename)
    (std::vector<cv::Point2f>, points)
)
// clang-format on

DRISHTI_BEGIN_NAMESPACE(FACE)

template <typename Iterator, typename Skipper = qi::blank_type>
struct helen_parser : qi::grammar<Iterator, HELEN::Table(), Skipper>
{
    helen_parser()
        : helen_parser::base_type(start)
    {
        static const char colsep = ',';

        key = qi::lexeme[+((qi::alnum | qi::char_('-') | qi::char_('_')) - qi::blank)];
        point = (qi::float_ >> colsep >> qi::float_);
        points = (point % colsep);
        line = key >> colsep >> points >> qi::eol;
        start = +line >> qi::eoi;

        BOOST_SPIRIT_DEBUG_NODES((start)(key)(points)(line))
    }

    qi::rule<Iterator, cv::Point2f(), Skipper> point;
    qi::rule<Iterator, std::vector<cv::Point2f>(), Skipper> points;
    qi::rule<Iterator, std::string(), Skipper> key;
    qi::rule<Iterator, HELEN::record(), Skipper> line;
    qi::rule<Iterator, HELEN::Table(), Skipper> start;
};

DRISHTI_END_NAMESPACE(FACE)

FACE::Table parseHELEN(const std::string& filename)
{
    FACE::Table table;

    // ranges (inclusive)
    static const int mouthL = 72;
    static const int mouthR = 58;
    static cv::Range le(114, 133);  // left eye
    static cv::Range re(134, 153);  // right eye
    static cv::Range nose(41, 57);  // nose
    static cv::Range lb(154, 173);  // left brow
    static cv::Range rb(174, 193);  // right brow
    static cv::Range mouth(58, 85); // mouth
    static cv::Range sider(0, 3);
    static cv::Range sidel(36, 39);

    table.eyeL.resize(le.end - le.start + 1);
    table.eyeR.resize(re.end - re.start + 1);
    table.nose.resize(nose.end - nose.start + 1);
    table.mouthL = { mouthL };
    table.mouthR = { mouthR };

    table.browL.resize(lb.end - lb.start + 1);
    table.browR.resize(rb.end - rb.start + 1);
    table.mouth.resize(mouth.end - mouth.start + 1);
    table.sideL.resize(sider.end - sider.start + 1);
    table.sideR.resize(sidel.end - sidel.start + 1);

    std::iota(table.eyeL.begin(), table.eyeL.end(), le.start);
    std::iota(table.eyeR.begin(), table.eyeR.end(), re.start);
    std::iota(table.nose.begin(), table.nose.end(), nose.start);
    std::iota(table.browL.begin(), table.browL.end(), lb.start);
    std::iota(table.browR.begin(), table.browR.end(), rb.start);
    std::iota(table.mouth.begin(), table.mouth.end(), mouth.start);

    std::iota(table.sideR.begin(), table.sideR.end(), sider.start);
    std::iota(table.sideL.begin(), table.sideL.end(), sidel.start);

    // 12 upper points
    //table.eyeL = {114, 120, 125, 129 }; // clockwise
    //table.eyeR = {134, 140, 145, 149 }; // conouter clockwise

    std::ifstream is(filename);
    if (is.is_open())
    {
        HELEN::Table output;

        is.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator begin(is), end;
        FACE::helen_parser<decltype(begin)> parser;
        bool success = qi::phrase_parse(begin, end, parser, qi::blank, output);

        table.lines.resize(output.lines.size());
        for (int i = 0; i < table.lines.size(); i++)
        {
            table.lines[i].filename = output.lines[i].filename;
            table.lines[i].points = output.lines[i].points;
        }
    }

    return table;
}
