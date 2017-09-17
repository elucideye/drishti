/*! -*-c++-*-
  @file   drishti_cli.h
  @author David Hirvonen
  @brief  Portable cli utilites with no dependnecies

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  This file is intended to support common command line operations without
  any external dependencies, other than C++11 system libraries.  The file
  should be self contained such that it can be installed with a public
  sdk sample application.

*/

#ifndef drishti_cli_h
#define drishti_cli_h

#include "drishti/core/Line.h"

#ifndef DRISHTI_BEGIN_NAMESPACE
// clang-format off
#define DRISHTI_BEGIN_NAMESPACE(X) namespace X {
#define DRISHTI_END_NAMESPACE(X) }
// clang-format on
#endif

#include <fstream>
#include <vector>
#include <string>

DRISHTI_BEGIN_NAMESPACE(drishti) // namespace drishti {
DRISHTI_BEGIN_NAMESPACE(cli)     // namespace cli {

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

// drishti::cli::
inline std::vector<std::string>
expand(const std::string& filename, const std::string& ext = ".txt")
{
    std::vector<std::string> list;
    if (filename.find(ext) != std::string::npos)
    {
        std::ifstream ifs(filename.c_str());
        if (ifs)
        {
            auto iter = std::istream_iterator<drishti::core::Line>(ifs);
            std::copy(iter, {}, std::back_inserter(list));
        }
    }
    else
    {
        list = { filename };
    }
    return list;
}

// drishti::cli::file::
DRISHTI_BEGIN_NAMESPACE(file) // namespace file {
inline bool exists(const std::string& filename)
{
    return std::ifstream(filename).good();
}
DRISHTI_END_NAMESPACE(file) // }

// drishti::cli::directory
DRISHTI_BEGIN_NAMESPACE(directory) // namespace directory {
// Options:
// * c++17 std::filesystem (not adopted yet)
// * experimental c++14 (not adopted yet)
// * boost::filesystem::exists()
// * stat()
// * create a small test file <= ### FOR NOW
// Check for writeable directory (will create .drishti file as side-effect)
inline bool exists(const std::string& dir, const std::string& tag = ".drishti")
{
    return std::ofstream(dir + "/" + tag).good();
}
DRISHTI_END_NAMESPACE(directory) // }

DRISHTI_END_NAMESPACE(cli) // }

DRISHTI_END_NAMESPACE(drishti) // }

#endif // drishti_cli_h
