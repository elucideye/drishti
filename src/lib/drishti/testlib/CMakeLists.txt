hunter_add_package(Boost COMPONENTS regex)
find_package(Boost CONFIG REQUIRED regex)

add_library(drishti_test_main drishti_test_main.cpp)
target_link_libraries(drishti_test_main PUBLIC Boost::regex)
