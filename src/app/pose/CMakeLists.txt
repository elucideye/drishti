set(test_app eos-pose)
set(test_sources pose.cpp)

add_executable(${test_app} ${test_sources})
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})

message("DRISHTI_DO_GPU_TESTING = ${DRISHTI_DO_GPU_TESTING}")
if(${DRISHTI_DO_GPU_TESTING})
  target_link_libraries(${test_app} aglet::aglet)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_DO_GPU_TESTING=1)
endif()

set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
