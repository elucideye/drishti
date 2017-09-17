/*! -*-c++-*-
  @file   Line.h
  @author David Hirvonen
  @brief  Declaration of stream based line reader.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#ifndef __drishti_core_Line_h__
#define __drishti_core_Line_h__

#include "drishti/core/drishti_core.h"
#include <sstream>
#include <algorithm>
#include <fstream>
#include <iostream>
#include <numeric>
#include <vector>
#include <iterator>
#include <memory>
#include <iomanip>

DRISHTI_CORE_NAMESPACE_BEGIN

// For use with istream_iterator to read complete lines (new line delimiter)
// std::vector<std::string> lines;
// std::copy(std::istream_iterator<line>(std::cin), std::istream_iterator<line>(), std::back_inserter(lines));
//http://stackoverflow.com/questions/1567082/how-do-i-iterate-over-cin-line-by-line-in-c/1567703#1567703

class Line
{
    std::string data;

public:
    friend std::istream& operator>>(std::istream& is, Line& l)
    {
        std::getline(is, l.data);
        return is;
    }
    operator std::string() const
    {
        //std::string name = data.substr(0, data.find(','));
        //std::cout << name << std::endl;
        return data;
    }
};

// http://stackoverflow.com/a/19280868
struct csv_reader : std::ctype<char>
{
    csv_reader()
        : std::ctype<char>(get_table())
    {
    }
    static std::ctype_base::mask const* get_table()
    {
        static std::vector<std::ctype_base::mask> rc(table_size, std::ctype_base::mask());
        rc[','] = std::ctype_base::space;
        rc['\n'] = std::ctype_base::space;
        rc[' '] = std::ctype_base::space;
        return &rc[0];
    }
};

DRISHTI_CORE_NAMESPACE_END

#endif // __drishti_core_Line_h__
