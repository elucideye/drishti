/*! -*-c++-*-
  @file   fddb/FDDB.cpp
  @author David Hirvonen
  @brief  FDDB dataset parsing.

  \copyright Copyright 2015-2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "FDDB.h"
#include <iterator>

// clang-format off
BOOST_FUSION_ADAPT_STRUCT(
    FDDB::record,
    (std::string, filename)
    (std::vector<FDDB::record::EllipseEntry>, ellipses)
)
// clang-format on

// file = { record }
// record = filename <CR> integer <CR> { ellipse digit <CR> }
// filename = { alpha }
// ellipse = real real real real real

namespace FDDB
{

std::ostream& operator<<(std::ostream& os, const record& r)
{
    os << r.filename << std::endl;
    for (const auto& e : r.ellipses)
    {
        for (const auto& d : e.first)
        {
            os << d << ' ';
        }

        os << int(e.second) << std::endl;
    }
    return os;
}

template <typename Iterator, typename Skipper = qi::blank_type>
struct record_parser : qi::grammar<Iterator, std::vector<record>(), Skipper>
{
    record_parser()
        : record_parser::base_type(start)
    {
        ellipse = qi::repeat(5)[(qi::double_ % ' ')] >> qi::int_ >> (qi::eol | qi::eoi);
        ellipses = +ellipse;
        entry = +(qi::char_ - qi::eol) >> qi::eol >> qi::omit[qi::int_] >> qi::eol >> ellipses;
        start = +entry;

        //BOOST_SPIRIT_DEBUG_NODE( ellipse );
        //BOOST_SPIRIT_DEBUG_NODE( ellipses );
        //BOOST_SPIRIT_DEBUG_NODE( entry );
        //BOOST_SPIRIT_DEBUG_NODE( start );
    }
    qi::rule<Iterator, FDDB::record::EllipseEntry(), Skipper> ellipse;
    qi::rule<Iterator, std::vector<FDDB::record::EllipseEntry>(), Skipper> ellipses;
    qi::rule<Iterator, record(), Skipper> entry;
    qi::rule<Iterator, std::vector<record>(), Skipper> start;
};
}

std::vector<FDDB::record> parseFDDB(const std::string& filename)
{
    std::vector<FDDB::record> result;

    // open file, disable skipping of whitespace
    std::ifstream file(filename);

    if (file)
    {
        file.unsetf(std::ios::skipws);
        boost::spirit::istream_iterator begin(file), end;
        FDDB::record_parser<decltype(begin)> parser;

        bool success = qi::phrase_parse(begin, end, parser, qi::blank, result);
        if (success)
            for (const auto& r : result)
                std::cout << r << " " << std::endl;
    }

    return result;
}

// FDDB inline test string
//const char *sFDDB = R"fddb(2002/08/11/big/img_591
//1
//123.583300 85.549500 1.265839 269.693400 161.781200  1
//2002/08/26/big/img_265
//3
//67.363819 44.511485 -1.476417 105.249970 87.209036  1
//41.936870 27.064477 1.471906 184.070915 129.345601  1
//70.993052 43.355200 1.370217 340.894300 117.498951  1
//)fddb";
