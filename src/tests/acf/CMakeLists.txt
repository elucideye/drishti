# Copyright (c) 2015, Ruslan Baratov, David Hirvonen
# All rights reserved.

set(SOURCES
  test-shader.cpp
  QGLContext.h
  QGLContext.cpp
  test-acf.cpp
)

add_executable(test-acf ${SOURCES})

target_link_libraries(test-acf
  Qt5::Widgets
  ${OpenCV_LIBS}
  GTest::main
  ${OGLES_GPGPU_LIB}
  drishtisdk
  )

find_package(Qt5PrintSupport REQUIRED)
target_link_libraries(test-acf Qt5::PrintSupport)
set_property(TARGET test-acf PROPERTY FOLDER "app/tests")

##
## GTest + CTest
##

enable_testing()
add_test(acf_test test-acf
  "${CMAKE_SOURCE_DIR}/assets/images/lena512color.png"
  "${CMAKE_SOURCE_DIR}/assets/images/lena512gray.png"
  )

