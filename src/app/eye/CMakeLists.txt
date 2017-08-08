add_executable(drishti-eye eye.cpp)
target_link_libraries(drishti-eye drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})
set_property(TARGET drishti-eye PROPERTY FOLDER "app/console")
install(TARGETS drishti-eye DESTINATION bin)

add_executable(drishti-eye-serialize serialize.cpp)
target_link_libraries(drishti-eye-serialize drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})
set_property(TARGET drishti-eye-serialize PROPERTY FOLDER "app/console")
install(TARGETS drishti-eye-serialize DESTINATION bin)
