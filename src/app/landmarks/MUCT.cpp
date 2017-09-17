/*! -*-c++-*-
 @file   MUCT.cpp
 @author David Hirvonen
 @brief  High level routines for parsing MUCT data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

//#define BOOST_SPIRIT_DEBUG

#include "landmarks/MUCT.h"

#include <fstream>

namespace qi = boost::spirit::qi;

// muct/muct-landmarks/muct76-opencv.csv
//name,tag,x00,y00,...,x76,y75
//i000qa-fn,0000,201,348,...,340,340.5

DRISHTI_BEGIN_NAMESPACE(FACE)

template <typename Iterator, typename Skipper = qi::blank_type>
struct muct_parser : qi::grammar<Iterator, FACE::Table(), Skipper>
{
    muct_parser()
        : muct_parser::base_type(start)
    {
        static const char colsep = ',';

        key = qi::lexeme[+((qi::alnum | qi::char_('-')) - qi::blank)];
        point = (qi::float_ >> colsep >> qi::float_);
        line = key >> colsep >> qi::int_ >> colsep >> (point % colsep) >> qi::eol;
        header = (key % colsep) >> qi::eol;
        start = header >> +line >> qi::eoi;

        BOOST_SPIRIT_DEBUG_NODES((start)(header)(key)(line));
    }

    qi::rule<Iterator, std::string(), Skipper> key;
    qi::rule<Iterator, cv::Point2f(), Skipper> point;
    qi::rule<Iterator, FACE::record(), Skipper> line;
    qi::rule<Iterator, std::vector<std::string>(), Skipper> header;
    qi::rule<Iterator, FACE::Table(), Skipper> start;
};

DRISHTI_END_NAMESPACE(FACE)

// 15-20     => left eyebrow
// 21-26     => right eyebrow
// 27-30     => right eye
// 31        => right pupil
// 32-35     => left eye
// 36        => right eye
// 37-45     => nose

FACE::Table parseMUCT(const std::string& filename)
{
    FACE::Table table;
    table.eyeR = { 27, 29 };
    table.eyeL = { 34, 32 };
    table.nose = { 46, 47, 67 };
    table.brow = { 24, 18 };
    table.mouth = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 };

    //table.eyeL = { 32, 33, 34, 35 };
    //table.eyeR = { 27, 28, 29, 30 };
    //table.nose = { 37, 38, 39, 40, 41, 42, 43, 44, 45 };

    std::ifstream is(filename);
    if (is.is_open())
    {
        is.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator begin(is), end;
        FACE::muct_parser<decltype(begin)> parser;
        bool success = qi::phrase_parse(begin, end, parser, qi::blank, table);
    }
    return table;
}
