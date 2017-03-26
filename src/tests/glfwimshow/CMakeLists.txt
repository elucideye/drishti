set(test_app test-glfwimshow)

add_executable(${test_app} glfwimshow.cpp)
target_link_libraries(${test_app} PUBLIC ${OpenCV_LIBS} imshow::imshow)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")
