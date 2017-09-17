/*! -*-c++-*-
  @file   serialize
  @author David Hirvonen
  @brief  Serialize an eye model.

  \copyright Copyright 2017 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/eye/EyeModelEstimator.h"
#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_cereal_pba.h"

#include "cxxopts.hpp"

// clang-format off
#if defined(DRISHTI_USE_IMSHOW)
#  include "imshow/imshow.h"
#endif
// clang-format on

#include <fstream>

int gauze_main(int argc, char* argv[])
{
    const auto argumentCount = argc;

    // Instantiate line logger:
    auto logger = drishti::core::Logger::create("drishti-eye-serialize");

    std::string sEye;
    std::string sIris;
    std::string sPupil;
    std::string sOutput; // output eye model

    cxxopts::Options options("drishti-eye-serialize", "Serialize eye model from list of regressors");

    // clang-format off
    options.add_options()
        ("e,eye", "Eye regressor", cxxopts::value<std::string>(sEye))
        ("i,iris", "Iris regressor", cxxopts::value<std::string>(sIris))
        ("p,pupil", "Pupil regressor", cxxopts::value<std::string>(sPupil))
        ("o,output", "Output eye model files", cxxopts::value<std::string>(sOutput))        
        ("h,help", "Print help message");
    // clang-format on    

    options.parse(argc, argv);

    if((argumentCount <= 1) || options.count("help"))
    {
        std::cout << options.help({""}) << std::endl;
        return 0;
    }

    { // Test that we can write to the specified output model before starting:
        std::ofstream check(sOutput);
        if(check)
        {
            remove(sOutput.c_str());
        }
        else
        {
            logger->error("Cannot create specified output model {}", sOutput);
            return 1;
        }
    }    

    drishti::eye::EyeModelEstimator model({sEye, sIris, sPupil});
    save_cpb(sOutput, model);
    
    drishti::eye::EyeModelEstimator dummy;
    load_cpb(sOutput, dummy);

    return 0;
}

int main(int argc, char **argv)
{
    try
    {
        return gauze_main(argc, argv);
    }
    catch (std::exception &e)
    {
        std::cerr << "Exception: " << e.what() << std::endl;
        return 1;
    }

}
