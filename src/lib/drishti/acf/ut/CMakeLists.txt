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
  test-Detector.cpp
  test-drishti-acf.cpp
  )

if(DRISHTI_ACF_DO_GPU)
  list(APPEND SOURCES QGLContext.h QGLContext.cpp)
endif()

add_executable(test-drishti-acf ${SOURCES})

target_link_libraries(test-drishti-acf
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk
  ${DRISHTI_OPENGL_LIBS}
  )

if(DRISHTI_ACF_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)  
  target_link_libraries(test-drishti-acf ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(test-drishti-acf PUBLIC DRISHTI_ACF_DO_GPU=${DRISHTI_ACF_DO_GPU})
endif()

set_property(TARGET test-drishti-acf PROPERTY FOLDER "app/tests")

##
## GTest + CTest
##

if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner_48x48.cpb")  
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.cpb")  
elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner_48x48.pba.z")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.pba.z")
elseif(DRISHTI_SERIALIZE_WITH_CVMATIO AND EXISTS "${assets_dir}/drishti_face_inner_48x48.mat")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.mat")
else()
  message(FATAL_ERROR "Failed to find suitable ACF detector")
endif()

if(NOT EXISTS "${assets_dir}/images/lena512color.png")
  message(FATAL_ERROR "Failed to find test face image: ${assets_dir}/images/lena512color.png") 
endif()

enable_testing()
add_test(NAME DrishtiACFTest COMMAND test-drishti-acf
  "${assets_dir}/images/lena512color.png"
  "${assets_dir}/images/lena512gray.png"
  "${DRISHTI_ACF_FACE_MODEL}"
  "${CMAKE_CURRENT_BINARY_DIR}" # Used as portable temp folder
  )
