#### opencv_size ####
add_executable(opencv_size opencv_size.cpp)
target_link_libraries(opencv_size ${OpenCV_LIBS})
install(TARGETS opencv_size DESTINATION bin)
set_property(TARGET opencv_size PROPERTY FOLDER "app/benchmarks")
