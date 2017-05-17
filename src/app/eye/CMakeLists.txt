set(test_app drishti-eye)

add_executable(${test_app} eye.cpp)
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})
set_target_properties(${test_app} PROPERTIES FOLDER "app/console" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
install(TARGETS ${test_app} DESTINATION bin)
