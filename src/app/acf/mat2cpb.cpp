/*!
 @file   mat2cpb.cpp
 @author David Hirvonen
 @brief  Convert drishti ACF detection model from MAT (matlab) to CPB (cereal)
 
 \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
 \license{This project is released under the 3 Clause BSD License.}
 
 */

#include "drishti/core/Logger.h"
#include "drishti/core/drishti_string_hash.h"
#include "drishti/testlib/drishti_cli.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"
#include "drishti/acf/ACF.h"

#include <cereal/archives/json.hpp>

#include "rapidxml/rapidxml.hpp"
#include "rapidxml/rapidxml_print.hpp"

#include "cxxopts.hpp"

#include <fstream>
#include <iostream>
#include <string>

int gauze_main(int argc, char** argv)
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("acf-mat2cpb");

    // ############################
    // ### Command line parsing ###
    // ############################

    std::string sInput, sOutput;

    cxxopts::Options options("acf-mat2cpb", "Convert MAT to CPB format");

    // clang-format off
    options.add_options()
        ("i,input", "Input file", cxxopts::value<std::string>(sInput))
        ("o,output", "Output directory", cxxopts::value<std::string>(sOutput))
        ("h,help", "Print help message");
    // clang-format on

    options.parse(argc, argv);

    if ((argumentCount <= 1) || options.count("help"))
    {
        logger->info("{}", options.help({ "" }));
        return 0;
    }

    if (sInput.empty())
    {
        logger->error("Must specify input MAT file");
        return 1;
    }

    if (sOutput.empty())
    {
        logger->error("Must specify output CPB file");
    }
    else
    {
        std::ofstream ofs(sOutput);
        if(ofs)
        {
            remove(sOutput.c_str());
        }
        else
        {
            logger->error("Unable to open {} for writing", sOutput);
        }
    }
    
    drishti::acf::Detector acf(sInput);

    save_cpb(sOutput, acf);
}

int main(int argc, char** argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch (std::exception& e)
    {
        std::cerr << e.what() << std::endl;
    }
}
