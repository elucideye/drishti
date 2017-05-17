##############
### common ###
##############

include(drishti_set_unit_test_assets)
# Use static lib to avoid dynamic frameworks in internal tests
if(IOS)
  set(drishti_sdk_lib drishti_static)
else()
  set(drishti_sdk_lib drishti)
endif()

drishti_set_unit_test_assets(
  DRISHTI_ACF_FACE_MODEL
  DRISHTI_FACE_LANDMARKER
  DRISHTI_EYE_MODEL
  DRISHTI_MEAN_FACE_LANDMARKS
  )

message("DRISHTI_EYE_MODEL : ${DRISHTI_EYE_MODEL}")

enable_testing()

###########
### Eye ###
###########

set(e_test_name DrishtiDrishtiEyeTest)
set(e_test_app test-drishti-drishti-eye)

add_executable(${e_test_app} test-EyeSegmenter.cpp)
set_target_properties(${e_test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
target_link_libraries(${e_test_app} PUBLIC ${drishti_sdk_lib}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )

if(DRISHTI_BUILD_C_INTERFACE)
  target_link_libraries(${e_test_app} PUBLIC drishti_c)
  target_compile_definitions(${e_test_app} PUBLIC DRISHTI_BUILD_C_INTERFACE)
endif()

set(DRISHTI_TEST_EYE_IMAGE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_JSON "${assets_dir}/images/2318-eye.json")

drishti_add_test(
  NAME ${e_test_name}
  COMMAND ${e_test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_IMAGE}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_JSON}>"
  "0"
  )

if(WIN32)
  set_property(
    TEST ${e_test_name}
    PROPERTY
    ENVIRONMENT
    "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
    )
endif()

############
### Face ###
############

# Note: Currently we will skip the public FACE api if GPU is unavailable
# but we will leave conditionals in place below in anticipation of supporting
# a CPU version of the public SDK:
if(NOT DRISHTI_BUILD_HCI OR NOT DRISHTI_HCI_DO_GPU)
  return()
endif()

set(DRISHTI_TEST_FACE_IMAGE_COLOR "${assets_dir}/images/lena512color.png")

set(f_test_name DrishtiDrishtiFaceTest)
set(f_test_app test-drishti-drishti-face)

set(f_sources test-FaceTracker.cpp)
if(DRISHTI_HCI_DO_GPU)
  list(APPEND f_sources ${DRISHTI_QTPLUS_HDRS} ${DRISHTI_QTPLUS_SRCS})
endif()

add_executable(${f_test_app} ${f_sources})

set_target_properties(${f_test_app} PROPERTIES FOLDER "app/tests" INTERPROCEDURAL_OPTIMIZATION ${DRISHTI_EXE_IPO})
target_link_libraries(${f_test_app} PUBLIC ${drishti_sdk_lib}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::face::FaceModel serialization
  )

drishti_bool_to_int(DRISHTI_BUILD_QT build_qt)
target_compile_definitions(${f_test_app} PUBLIC DRISHTI_BUILD_QT=${build_qt})

### POST_BUILD GPU ADDITIONS ###
if(DRISHTI_HCI_DO_GPU)
  find_package(Qt5PrintSupport REQUIRED)
  target_link_libraries(${f_test_app} PUBLIC ${OGLES_GPGPU_LIB} Qt5::Widgets Qt5::PrintSupport)
  target_compile_definitions(${f_test_app} PUBLIC DRISHTI_HCI_DO_GPU=${DRISHTI_HCI_DO_GPU})
endif()

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(${f_test_app} PUBLIC
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(${f_test_app} PUBLIC
      "-framework ImageIO"
      "-framework CoreVideo"
      )
  endif()
endif()

###

if(DRISHTI_BUILD_C_INTERFACE)
  target_link_libraries(${f_test_app} PUBLIC drishti_c)
  target_compile_definitions(${f_test_app} PUBLIC DRISHTI_BUILD_C_INTERFACE)
endif()

drishti_add_test(
  NAME ${f_test_name}
  COMMAND ${f_test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_ACF_FACE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_MEAN_FACE_LANDMARKS}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_FACE_LANDMARKER}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_FACE_IMAGE_COLOR}>"  
  )

if(WIN32)
  set_property(
    TEST ${f_test_name}
    PROPERTY
    ENVIRONMENT
    "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
    )
endif()
