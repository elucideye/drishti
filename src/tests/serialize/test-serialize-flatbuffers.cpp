#include "Monster_c.h"
#include "monster_generated.h" // Already includes "flatbuffers/flatbuffers.h".
#include <iostream>
#include <fstream>

using namespace MyGame::Sample;

int drishti_main(int argc, char **argv)
{
    if(argc != 2)
    {
        std::cerr << "Usage: " << argv[0] << " filename.out" << std::endl;
        exit(1);
    }
    
    flatbuffers::FlatBufferBuilder builder;

    drishti::Monster monsterSrc = drishti::createMonster();
    monsterSrc.serialize(builder);
    
    std::ofstream output(argv[1], std::ios::binary);
    if(output)
    {
        std::cout << "Flatbuffer writing: " << builder.GetSize() << " bytes " << std::endl;
        output.write((const char *)builder.GetBufferPointer(), builder.GetSize());
    }
    
    drishti::Monster monsterDst;
    monsterDst.deserialize(builder.GetBufferPointer());

    return 0;
}
