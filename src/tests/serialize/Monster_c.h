#ifndef MONSTER_H
#define MONSTER_H

#include <opencv2/core.hpp>

#include <iterator>
#include <memory>

#if defined(DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
#  include <flatbuffers/flatbuffers.h>
#elif defined(DRISHTI_USE_BOOST_SERIALIZATION)
#  include <boost/archive/binary_oarchive.hpp>
#  include <boost/archive/binary_iarchive.hpp>
#  include <boost/iostreams/filtering_stream.hpp>
#  include <boost/iostreams/filtering_streambuf.hpp>
#  include <boost/iostreams/filter/gzip.hpp>
#  include <boost/iostreams/filter/bzip2.hpp>
#  include <boost/iostreams/copy.hpp>
#  include <boost/serialization/vector.hpp>
#elif defined(DRISHTI_USE_CEREAL_SERIALIZATION)
#  include <cereal/types/string.hpp>
#  include <cereal/types/vector.hpp>
#  include <cereal/types/memory.hpp>
#endif

// This provides an approximate C++ layout for monster.fbs

#if !defined(DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
namespace cv
{
    template<class Archive>
    void serialize(Archive & ar, cv::Vec3f & v, const unsigned int version)
    {
        ar & v[0];
        ar & v[1];
        ar & v[2];
    }
}
#endif

namespace drishti
{
    struct Weapon
    {
        Weapon() {}
        Weapon(const std::string &name, int16_t damage) : name(name), damage(damage) {}
        
        std::string name;
        int16_t damage;
        
#if !defined(DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
        template <class Archive> void serialize(Archive & ar, const unsigned int version)
        {
            ar & name;
            ar & damage;
        }
#endif
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
        
#if defined(DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
        void serialize(flatbuffers::FlatBufferBuilder &builder);
        void deserialize(unsigned char *buffer);
#else
        template <class Archive> void serialize(Archive & ar, const unsigned int version)
        {
            ar & pos;
            ar & mana;
            ar & hp;
            ar & name;
            ar & friendly;
            ar & inventory;
            ar & color;
            ar & weapons;
        }
#endif
    };

    inline Monster createMonster()
    {
        Monster monster;
        monster.color = 1;
        monster.hp = 2;
        monster.inventory = { 1, 2, 3, 4, 5 };
        monster.friendly = true;
        monster.mana = 4;
        monster.pos = { 1.f, 2.f, 3.f };
        monster.name = "Trogdor";
        monster.weapons = { {"axe", 4}, {"hammer", 3} };
        return monster;
    }
    
};

#endif // MONSTER_H
