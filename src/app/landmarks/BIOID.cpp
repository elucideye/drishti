/*! -*-c++-*-
 @file   BIOID.cpp
 @author David Hirvonen
 @brief  High level routines for parsing BIOID data.
 
 \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "landmarks/BIOID.h"

#include "drishti/core/Line.h"
#include "drishti/core/string_utils.h"
// ======= BIOID ========

#include <boost/spirit/include/qi.hpp>
#include <boost/spirit/include/phoenix.hpp>

namespace qi = boost::spirit::qi;
namespace phx = boost::phoenix;

DRISHTI_BEGIN_NAMESPACE(BIOID)

struct record
{
    std::vector<cv::Point2f> points;
};

DRISHTI_END_NAMESPACE(BIOID)

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    BIOID::record,
    (std::vector<cv::Point2f>, points)
)
// clang-format on

//There are 20 manually placed points on each of your 1521 images.
//The markup scheme is as follows:
//0 = right eye pupil
//1 = left eye pupil
//2 = right mouth corner
//3 = left mouth corner
//4 = outer end of right eye brow
//5 = inner end of right eye brow
//6 = inner end of left eye brow
//7 = outer end of left eye brow
//8 = right temple
//9 = outer corner of right eye
//10 = inner corner of right eye
//11 = inner corner of left eye
//12 = outer corner of left eye
//13 = left temple
//14 = tip of nose
//15 = right nostril
//16 = left nostril
//17 = centre point on outer edge of upper lip
//18 = centre point on outer edge of lower lip
//19 = tip of chin

//version: 1
//n_points: 20
//{
//    159.128 108.541
//    230.854 109.176
//    164.841 179.633
//    223.237 178.998
//    132.469 93.9421
//}

DRISHTI_BEGIN_NAMESPACE(FACE)

template <typename Iterator, typename Skipper = qi::blank_type>
struct bioid_parser : qi::grammar<Iterator, BIOID::record(), Skipper>
{
    bioid_parser()
        : bioid_parser::base_type(start)
    {
        point = (qi::float_ >> qi::float_) >> qi::eol;
        header = +(qi::char_ - qi::char_('{')) >> qi::char_('{') >> qi::eol;
        start = header >> +(point) >> qi::char_('}') >> qi::eoi;

        BOOST_SPIRIT_DEBUG_NODES((start)(header)(point));
    }

    qi::rule<Iterator, Skipper> header;
    qi::rule<Iterator, cv::Point2f(), Skipper> point;
    qi::rule<Iterator, BIOID::record(), Skipper> start;
};

DRISHTI_END_NAMESPACE(BIOID)

void parseBIOID(const std::string& filename, BIOID::record& output)
{
    std::ifstream is(filename);
    if (is.is_open())
    {
        is.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator begin(is), end;
        FACE::bioid_parser<decltype(begin)> parser;
        bool success = qi::phrase_parse(begin, end, parser, qi::blank, output);
    }
}

FACE::Table parseBIOID(const std::string& filename)
{
    using drishti::core::Line;

    FACE::Table table;
    table.eyeR = { 9, 10 };
    table.eyeL = { 11, 12 };
    table.nose = { 15, 16, 14 };
    table.brow = { 5, 6 };

    std::vector<std::pair<std::string, std::string>> filenames;

    std::vector<std::string> lines;
    std::ifstream file(filename.c_str());
    if (file.is_open())
    {
        std::copy(std::istream_iterator<Line>(file), std::istream_iterator<Line>(), std::back_inserter(lines));
        for (auto& l : lines)
        {
            std::stringstream iss(l);
            std::vector<std::string> tokens;
            std::copy(std::istream_iterator<std::string>(iss), std::istream_iterator<std::string>(), std::back_inserter(tokens));

            //for(auto &t : tokens) std::cout << t << " "; std::cout << std::endl;

            if (tokens.size() == 2)
            {
                BIOID::record record;

                //std::cout << tokens[1] << std::endl;
                parseBIOID(tokens[1], record);

                //for(auto &p : record.points) std::cout << p << std::endl;

                FACE::record record_;
                record_.filename = tokens[0];
                record_.points = record.points;
                table.lines.push_back(record_);
            }
        }
    }

    if (table.lines.front().points.size() == 68)
    {
        table.browR = { 17, 18, 19, 20, 21 };
        table.browL = { 22, 23, 24, 25, 26 };
        table.eyeR = { 36, 37, 38, 39, 40, 41 };
        table.eyeL = { 42, 43, 44, 45, 46, 47 };
        table.nose = { 27, 28, 29, 30, 31, 32, 33, 34, 35 };
        table.mouth = { 48, 49, 50, 51, 52, 53, 54, 55, 56, 57, 58, 59 };
        table.mouthR = { 48 };
        table.mouthL = { 54 };
        table.brow = {};
    }

    return table;
}
