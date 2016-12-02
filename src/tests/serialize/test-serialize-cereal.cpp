#include "drishti/core/drishti_stdlib_string.h"
#include "drishti/core/drishti_serialize.h"
#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>
#include <cereal/types/memory.hpp>

#include "Monster_c.h"

#include <memory>
#include <fstream>
#include <iostream>

// http://uscilab.github.io/cereal/serialization_archives.html
//#include <cereal/archives/binary.hpp>
#include <cereal/archives/portable_binary.hpp>

#include "drishti/core/drishti_cereal_pba.h"

typedef cereal::PortableBinaryOutputArchive3 OArchive;
typedef cereal::PortableBinaryInputArchive3 IArchive;

struct Bar
{
    template <typename Archive> void serialize(Archive &ar, const std::uint32_t version)
    {
        std::int16_t tmp = foo;
        if(Archive::is_loading::value)
        {
            ar & tmp;
            foo = tmp;
        }
        else
        {
            tmp = foo;
            ar & tmp;
        }
        ar & foo;
    }
    std::int32_t foo;
};

int main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " filename.out" << std::endl;
        exit(1);
    }
    
    std::string filename = argv[1];

    { // Dump the record:
        drishti::Monster monsterSrc = drishti::createMonster();
        
        std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
        if(ofs)
        {
            OArchive oa(ofs);
            oa(monsterSrc);
        }
    }

    { // Load the record
        drishti::Monster monsterDst;

        std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
        if(ifs)
        {
            IArchive ia(ifs);
            ia(monsterDst);
            //ia >> monsterDst;
        }
    }
}
