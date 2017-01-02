/*!
  @file   main.cpp
  @author David Hirvonen
  @brief  Declaration of main routine with tilde expansion.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  To be used a source template for unit testing.
  Assumes test_main(int argc, char **argv) is defined.

*/

#include <stdlib.h>

#ifdef DRISHTI_HAVE_REGEX_H
#  include <regex>
int main(int argc, char** argv)
{
    std::string home = getenv("HOME");
    std::vector<std::string> args(argc); // provide storage for expanded arguments
    std::vector<char *> ptrs(argc);
    for(int i = 0; i < argc; i++)
    {
        args[i] = std::regex_replace(argv[i], std::regex("~/"), home + "/");
        ptrs[i] = const_cast<char *>(args[i].c_str());
    }
    return test_main(argc, &ptrs[0]);
}
#else
int main(int argc, char **argv)
{
    return test_main(argc, argv);
}
#endif

