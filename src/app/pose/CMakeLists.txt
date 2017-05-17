set(test_app eos-pose)

set(test_sources pose.cpp)
add_executable(${test_app} ${test_sources})
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})

if(DRISHTI_USE_IMSHOW)
  target_link_libraries(${test_app} imshow::imshow)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_USE_IMSHOW)
endif()

set_target_properties(${test_app} PROPERTIES FOLDER "app/console" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
install(TARGETS ${test_app} DESTINATION bin)
