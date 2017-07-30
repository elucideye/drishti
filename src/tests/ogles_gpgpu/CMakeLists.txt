set(test_name OglesGpgpuTest)
set(test_app test-ogles_gpgpu)

add_executable(${test_app} test-ogles_gpgpu.cpp)
target_link_libraries(${test_app} PUBLIC ogles_gpgpu::ogles_gpgpu aglet::aglet ${OpenCV_LIBS} GTest::gtest)
if(DRISHTI_OPENGL_ES3)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_OPENGL_ES3=1)
endif()  

set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  )
