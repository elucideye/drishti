/*!
  @file   pba2xml.cpp
  @author David Hirvonen
  @brief  Convert pba.z (compressed portable binary archives) to xml

  \copyright Copyright 2014-2016 Elucideye, Inc. All rights reserved.
  \license{This project is released under the 3 Clause BSD License.}

*/

#include "core/serialization.h"
#include "core/boost_serialize_common.h"
#include "eye/EyeModelEstimator.h"

#include <boost/archive/xml_iarchive.hpp>
#include <boost/archive/xml_oarchive.hpp>

#include <memory>

int main(int argc, char **argv)
{
    std::string sInput = argv[1];
    std::string sOutput = argv[2];
    
    auto sp = std::make_shared<DRISHTI_EYE::EyeModelEstimator>();

    { // Load the model from pba.z
        load_pba_z(sInput, *sp);
    }

    {
        //boost::iostreams::filtering_stream<boost::iostreams::output> buffer;
        //buffer.push(boost::iostreams::zlib_compressor(boost::iostreams::zlib::best_compression));
        //buffer.push(os);

        std::ofstream ofs(sOutput);
        assert(ofs.good());
        boost::archive::xml_oarchive oa(ofs);
        oa << BOOST_SERIALIZATION_NVP(*sp);
    }
}
