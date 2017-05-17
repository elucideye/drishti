#### opencv_size ####
set(app_name drishti_benchmark_opencv_size)
  
add_executable(${app_name} opencv_size.cpp)
target_link_libraries(${app_name} ${OpenCV_LIBS})
install(TARGETS ${app_name} DESTINATION bin)
set_target_properties(${app_name} PROPERTIES FOLDER "app/benchmarks" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
