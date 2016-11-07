cmake_minimum_required(VERSION 3.0)

hunter_add_package(flatbuffers)
find_package(flatbuffers CONFIG REQUIRED)

add_executable(flatbuffers-test flatbuffers-test.cpp)
target_link_libraries(flatbuffers-test flatbuffers::flatbuffers)
