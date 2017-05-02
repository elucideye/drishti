#include "Monster_c.h"

#include "monster_generated.h" // Already includes "flatbuffers/flatbuffers.h".

#include <flatbuffers/flatbuffers.h>

namespace drishti
{
void Monster::serialize(flatbuffers::FlatBufferBuilder& builder)
{
    // Create a FlatBuffer's `vector` from the `std::vector`.
    std::vector<flatbuffers::Offset<MyGame::Sample::Weapon>> weapons_;
    if (weapons.size())
    {
        for (auto& w : weapons)
        {
            weapons_.push_back(MyGame::Sample::CreateWeapon(builder, builder.CreateString(w.name), w.damage));
        }
    }

    auto weapons_vector_ = builder.CreateVector(weapons_);

    flatbuffers::Offset<flatbuffers::Vector<uint8_t>> inventory_;
    if (!inventory.empty())
    {
        inventory_ = builder.CreateVector(inventory);
    }

    flatbuffers::Offset<flatbuffers::String> name_;
    if (!name.empty())
    {
        name_ = builder.CreateString(name);
    }

    MyGame::Sample::MonsterBuilder monsterBuilder(builder);

    auto pos_ = MyGame::Sample::Vec3(pos[0], pos[1], pos[1]);
    monsterBuilder.add_pos(&pos_);
    monsterBuilder.add_name(name_);
    monsterBuilder.add_inventory(inventory_);
    monsterBuilder.add_hp(hp);
    monsterBuilder.add_mana(mana);
    monsterBuilder.add_color(MyGame::Sample::Color_Red);
    monsterBuilder.add_weapons(weapons_vector_);

    //monsterBuilder.add_equipped(<#flatbuffers::Offset<void> equipped#>);

    builder.Finish(monsterBuilder.Finish()); // Serialize the root of the object.
}

void Monster::deserialize(unsigned char* buffer)
{
    auto monster_ = MyGame::Sample::GetMonster(buffer);
    if (monster_ != nullptr)
    {
        auto weapons_ = monster_->weapons();
        if (weapons_ && weapons_->size())
        {
            for (const auto& w : *weapons_)
            {
                weapons.emplace_back(w->name()->str(), w->damage());
            }
        }

        const auto pos_ = monster_->pos();
        if (pos_ != nullptr)
        {
            pos = { pos_->x(), pos_->y(), pos_->z() };
        }

        auto name_ = monster_->name();
        if (name_ != nullptr)
        {
            name = monster_->name()->str();
        }

        auto inventory_ = monster_->inventory();
        if (inventory_ != nullptr)
        {
            std::copy(inventory_->begin(), inventory_->end(), std::back_inserter(inventory));
        }

        hp = monster_->hp();
        mana = monster_->mana();
        color = monster_->color();
    }
}
};
