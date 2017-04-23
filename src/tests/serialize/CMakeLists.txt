######################################
### Boost vs Cereal vs Flatbuffers ###
######################################

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
add_executable(tutorial-flatbuffers-monster ${FLATBUFFER_MONSTER_SRCS})
target_link_libraries(tutorial-flatbuffers-monster PUBLIC flatbuffers::flatbuffers ${OpenCV_LIBS})
target_compile_definitions(tutorial-flatbuffers-monster PUBLIC DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
set_property(TARGET tutorial-flatbuffers-monster PROPERTY FOLDER "app/tutorial")

#################################
### Serialization comparison  ###
#################################

# Flatbuffers serialization code:
add_executable(test-serialize-flatbuffers test-serialize-flatbuffers.cpp Monster_c.h Monster_c.cpp monster_generated.h)
target_link_libraries(test-serialize-flatbuffers PUBLIC flatbuffers::flatbuffers ${OpenCV_LIBS})
target_compile_definitions(test-serialize-flatbuffers PUBLIC DRISHTI_USE_FLAT_BUFFERS_SERIALIZATION)
target_include_directories(test-serialize-flatbuffers PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
#install(TARGETS test-serialize-flatbuffers DESTINATION bin)
set_property(TARGET test-serialize-flatbuffers PROPERTY FOLDER "app/tests")

# Boost serialization code:
add_executable(test-serialize-boost test-serialize-boost.cpp Monster_c.h)
target_link_libraries(test-serialize-boost PUBLIC Boost::iostreams Boost::serialization ${OpenCV_LIBS})
target_compile_definitions(test-serialize-boost PUBLIC DRISHTI_USE_BOOST_SERIALIZATION)
target_include_directories(test-serialize-boost PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
#install(TARGETS test-serialize-boost DESTINATION bin)
set_property(TARGET test-serialize-boost PROPERTY FOLDER "app/tests")

# Cereal serialization code:
add_executable(test-serialize-cereal test-serialize-cereal.cpp Monster_c.h)
target_link_libraries(test-serialize-cereal PUBLIC cereal::cereal ${OpenCV_LIBS})
target_compile_definitions(test-serialize-cereal PUBLIC DRISHTI_USE_CEREAL_SERIALIZATION)
target_include_directories(test-serialize-cereal PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
#install(TARGETS test-serialize-cereal DESTINATION bin)
set_property(TARGET test-serialize-cereal PROPERTY FOLDER "app/tests")

##
## GTest + CTest
##

enable_testing()

drishti_add_test(
  NAME DrishtiSerializationFlatbuffersTest
  COMMAND test-serialize-flatbuffers "./monster.fb" # output file
  )
drishti_add_test(
  NAME DrishtiSerializationCerealTest
  COMMAND test-serialize-cereal "./monster.cereal" # output file
  )
drishti_add_test(
  NAME DrishtiSerializationBoostTest
  COMMAND test-serialize-boost "./monster.pba.z" # output file
  )
