set(test_app drishti-acf)

add_executable(${test_app} acf.cpp)
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
