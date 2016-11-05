# Copyright (c) 2015, Ruslan Baratov, David Hirvonen
# All rights reserved.

# Currently uses QT for windows-less context (GPGPU stuff),
# but should also tests CPU based ACF code in isolation.

if(DRISHTI_BUILD_QT AND DRISHTI_BUILD_OGLES_GPGPU)
  set(DRISHTI_ACF_DO_GPU 1)
else()
  set(DRISHTI_ACF_DO_GPU 0)
endif()

set(SOURCES
  test-shader.cpp
  test-drishti-acf.cpp
  )

if(DRISHTI_ACF_DO_GPU)
  list(APPEND SOURCES QGLContext.h QGLContext.cpp)
endif()

add_executable(test-drishti-acf ${SOURCES})

target_link_libraries(test-drishti-acf
  ${OpenCV_LIBS}
  GTest::main
  drishtisdk
  )

if(DRISHTI_ACF_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)  
  target_link_libraries(test-drishti-acf ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
endif()

set_property(TARGET test-drishti-acf PROPERTY FOLDER "app/tests")

##
## GTest + CTest
##

enable_testing()
add_test(NAME acf_test COMMAND test-drishti-acf
  "${assets_dir}/images/lena512color.png"
  "${assets_dir}/images/lena512gray.png"
  "${assets_dir}/drishti_face_inner_48x48.mat"
  )
