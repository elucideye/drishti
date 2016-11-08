############################
### Boost vs Flatbuffers ###
############################

cmake_minimum_required(VERSION 3.0)

hunter_add_package(flatbuffers)
find_package(flatbuffers CONFIG REQUIRED)

set(FLATBUFFER_MONSTER_SRCS
  monster.cpp
  Monster_c.h
  Monster_c.cpp
  monster_generated.h
  )

# Flatbuffers tutorial code:
add_executable(monster ${FLATBUFFER_MONSTER_SRCS})
target_link_libraries(monster flatbuffers::flatbuffers)
target_compile_definitions(monster PUBLIC USE_FLAT_BUFFERS)
set_property(TARGET monster PROPERTY FOLDER "app/tests")

# Flatbuffers serialization code:
add_executable(flatbuffers-test flatbuffers-test.cpp Monster_c.h Monster_c.cpp monster_generated.h)
target_link_libraries(flatbuffers-test flatbuffers::flatbuffers)
target_compile_definitions(flatbuffers-test PUBLIC USE_FLAT_BUFFERS)
install(TARGETS flatbuffers-test DESTINATION bin)
set_property(TARGET flatbuffers-test PROPERTY FOLDER "app/tests")

# Boost serialization code:
add_executable(boost-serialize-test boost-serialize-test Monster_c.h)
target_link_libraries(boost-serialize-test Boost::iostreams Boost::serialization)
install(TARGETS boost-serialize-test DESTINATION bin)
set_property(TARGET boost-serialize-test PROPERTY FOLDER "app/tests")
