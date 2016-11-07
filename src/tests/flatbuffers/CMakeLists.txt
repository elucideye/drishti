cmake_minimum_required(VERSION 3.0)

hunter_add_package(flatbuffers)
find_package(flatbuffers CONFIG REQUIRED)

set(MONSTER_SRCS
  monster.cpp
  Monster_c.h
  Monster_c.cpp
  monster_generated.h
  )

add_executable(monster ${MONSTER_SRCS})
target_link_libraries(monster flatbuffers::flatbuffers)
set_property(TARGET monster PROPERTY FOLDER "app/tests")
