/*! -*-c++-*-
  @file   drishti_csv.h
  @author David Hirvonen
  @brief  Boost serialization CVS parser.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_drishti_csv_h__
#define __drishti_core_drishti_csv_h__

#include "drishti/core/drishti_core.h"
#include <boost/spirit/include/qi.hpp>

DRISHTI_CORE_NAMESPACE_BEGIN

//#define BOOST_SPIRIT_DEBUG
//
// http://stackoverflow.com/questions/18365463/how-to-parse-csv-using-boostspirit
// http://coliru.stacked-crooked.com/view?id=33b305c4fa7eb1f3bd133e829e3bf82e-93e6c6235a92d0c233f44beab03470ad

namespace qi = boost::spirit::qi;
using Column = std::string;
using Columns = std::vector<Column>;
using CsvLine = Columns;
using CsvFile = std::vector<CsvLine>;

template <typename It>
struct CsvGrammar : qi::grammar<It, CsvFile(), qi::blank_type>
{
    CsvGrammar()
        : CsvGrammar::base_type(start)
    {
        using namespace qi;

        static const char colsep = ',';

        start = -line % eol;
        line = column % colsep;
        column = quoted | *~char_(colsep);
        quoted = '"' >> *("\"\"" | ~char_('"')) >> '"';

        BOOST_SPIRIT_DEBUG_NODES((start)(line)(column)(quoted));
    }

private:
    qi::rule<It, CsvFile(), qi::blank_type> start;
    qi::rule<It, CsvLine(), qi::blank_type> line;
    qi::rule<It, Column(), qi::blank_type> column;
    qi::rule<It, std::string()> quoted;
};

// Example usage:
//int main()
//{
//    const std::string s = R"(1997,Ford,E350,"ac, abs, moon","""rusty""",3001.00)";
//
//    auto f(begin(s)), l(end(s));
//    CsvGrammar<std::string::const_iterator> p;
//
//    CsvFile parsed;
//    bool ok = qi::phrase_parse(f,l,p,qi::blank,parsed);
//
//    if (ok)
//    {
//        for(auto& line : parsed) {
//            for(auto& col : line)
//                std::cout << '[' << col << ']';
//            std::cout << std::endl;
//        }
//    } else
//    {
//        std::cout << "Parse failed\n";
//    }
//
//    if (f!=l)
//        std::cout << "Remaining unparsed: '" << std::string(f,l) << "'\n";
//}

DRISHTI_CORE_NAMESPACE_END

#endif
