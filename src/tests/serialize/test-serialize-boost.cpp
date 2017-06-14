#include "Monster_c.h"

#include <fstream>
#include <iostream>

int gauze_main(int argc, char** argv)
{
    if (argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " filename.out" << std::endl;
        exit(1);
    }

    std::string filename = argv[1];

    typedef boost::iostreams::zlib_compressor Compressor;
    typedef boost::iostreams::zlib_decompressor Decompressor;

    { // Dump the record:
        drishti::Monster monsterSrc = drishti::createMonster();

        std::ofstream ofs(filename, std::ios_base::out | std::ios_base::binary);
        if (ofs)
        {
            boost::iostreams::filtering_stream<boost::iostreams::output> buffer;
            buffer.push(Compressor());
            buffer.push(ofs);
            boost::archive::binary_oarchive oa(buffer);
            oa << monsterSrc;
        }
    }

    { // Load the record
        drishti::Monster monsterDst;
        std::ifstream ifs(filename, std::ios_base::in | std::ios_base::binary);
        if (ifs)
        {
            boost::iostreams::filtering_streambuf<boost::iostreams::input> buffer;
            buffer.push(Decompressor());
            buffer.push(ifs);
            boost::archive::binary_iarchive ia(buffer); // (ifs);
            ia >> monsterDst;
        }
    }

    return 0;
}
