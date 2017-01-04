/*!
  @file   main.cpp
  @author David Hirvonen
  @brief  Declaration of main routine with tilde expansion.

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

  To be used a source template for unit testing.
  Assumes test_main(int argc, char **argv) is defined.

*/

#include <iostream>
#include <stdlib.h>

int main(int argc, char** argv)
{
    std::string home = getenv("HOME");
    std::string path = home + "/";
    std::string pattern = "~/";

    std::vector<char *> ptrs(argc);    
    std::vector<std::string> args(argc); // provide storage for expanded arguments
    for(int i = 0; i < argc; i++)
    {
        args[i] = argv[i];

        auto position = args[i].find(pattern);
        if(position != std::string::npos)
        {
            args[i].replace(position, pattern.size(), path);
        }

        ptrs[i] = const_cast<char *>(args[i].c_str());
    }

    // Print argv[*] after tilde expansion:
    for(int i = 0; i < args.size(); i++)
    {
        std::cout << "argv[" << i << "] = " << args[i] << std::endl;
    }
    
    return test_main(argc, &ptrs[0]);
}
