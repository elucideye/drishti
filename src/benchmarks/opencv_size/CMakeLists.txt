#### opencv_size ####
set(app_name drishti_benchmark_opencv_size)
  
add_executable(${app_name} opencv_size.cpp)
target_link_libraries(${app_name} ${OpenCV_LIBS})
install(TARGETS ${app_name} DESTINATION bin)
set_property(TARGET ${app_name} PROPERTY FOLDER "app/benchmarks")
