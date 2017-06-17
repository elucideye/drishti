set(test_app eos-pose)
set(test_sources pose.cpp)

add_executable(${test_app} ${test_sources})
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})

if(TARGET drishti_gltest)
  target_link_libraries(${test_app} drishti_gltest)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_HAS_GLTEST)
endif()

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
