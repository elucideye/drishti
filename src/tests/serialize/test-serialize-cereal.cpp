#include <cereal/types/string.hpp>
#include <cereal/types/vector.hpp>

#include "Monster_c.h"

#include <fstream>
#include <iostream>

// http://uscilab.github.io/cereal/serialization_archives.html
//#include <cereal/archives/binary.hpp>
#include <cereal/archives/portable_binary.hpp>

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
            cereal::PortableBinaryOutputArchive oa(ofs);
            oa << monsterSrc;
        }
    }

    { // Load the record
        drishti::Monster monsterDst;

        std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
        if(ifs)
        {
            cereal::PortableBinaryInputArchive ia(ifs);
            ia >> monsterDst;
        }
    }
}
