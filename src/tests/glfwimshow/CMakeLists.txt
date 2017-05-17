set(test_app test-glfwimshow)

add_executable(${test_app} glfwimshow.cpp)
target_link_libraries(${test_app} PUBLIC ${OpenCV_LIBS} imshow::imshow)
set_target_properties(${test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
