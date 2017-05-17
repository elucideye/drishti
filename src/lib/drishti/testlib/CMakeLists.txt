hunter_add_package(Boost)
find_package(Boost CONFIG REQUIRED)

add_library(drishti_test_main drishti_test_main.cpp)
target_link_libraries(drishti_test_main PUBLIC Boost::boost)
set_target_properties(drishti_test_main PROPERTIES FOLDER "libs/drishti")
