set(test_app eos-pose)

set(test_sources
  pose.cpp
  convert.h
  FaceLandmarker.h
  FaceLandmarker.cpp
  )

add_executable(${test_app} ${test_sources})
target_link_libraries(${test_app} drishtisdk cxxopts::cxxopts ${OpenCV_LIBS})
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
