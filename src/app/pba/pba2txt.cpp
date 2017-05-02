/*!
  @file   pba2xml.cpp
  @author David Hirvonen
  @brief  Convert pba.z (compressed portable binary archives) to xml

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "drishti/core/boost_serialize_common.h"
#include "drishti/eye/EyeModelEstimator.h"

#include <boost/archive/text_iarchive.hpp>
#include <boost/archive/text_oarchive.hpp>

#include <memory>

int main(int argc, char** argv)
{
    std::string sInput = argv[1];
    std::string sOutput = argv[2];

    auto sp = std::make_shared<DRISHTI_EYE::EyeModelEstimator>();

    { // Load the model from pba.z
        load_pba_z(sInput, *sp);
    }

    {
        std::ofstream ofs(sOutput);
        assert(ofs.good());
        boost::archive::text_oarchive oa(ofs);
        oa << *sp;
    }
}
