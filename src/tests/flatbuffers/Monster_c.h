#ifndef MONSTER_H
#define MONSTER_H

#include <opencv2/core.hpp>

#include <flatbuffers/flatbuffers.h>

// This provides an approximate C++ layout for monster.fbs

namespace drishti
{
    struct Weapon
    {
        Weapon() {}
        Weapon(const std::string &name, int16_t damage) : name(name), damage(damage) {}
        
        std::string name;
        int16_t damage;
    };
    
    struct Monster
    {
        Monster() {}
        
        cv::Vec3f pos;
        int16_t mana;
        int16_t hp;
        std::string name;
        bool friendly;
        std::vector<uint8_t> inventory;
        unsigned char color;
        std::vector<Weapon> weapons;
        
        //std::unique_ptr<Equipment> equipped; // optional
        
        void serialize(flatbuffers::FlatBufferBuilder &builder);
        void deserialize(unsigned char *buffer);
    };
    
};

#endif // MONSTER_H
