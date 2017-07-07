##############
### common ###
##############

# Use static lib to avoid dynamic frameworks in internal tests
if(IOS)
  set(drishti_sdk_lib drishti_static)
else()
  set(drishti_sdk_lib drishti)
endif()

enable_testing()

###########
### Eye ###
###########

set(e_test_name DrishtiDrishtiEyeTest)
set(e_test_app test-drishti-drishti-eye)

add_executable(${e_test_app} test-EyeSegmenter.cpp)
set_property(TARGET ${e_test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${e_test_app} PUBLIC ${drishti_sdk_lib}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::eye::EyeModel serialization
  )

if(DRISHTI_BUILD_C_INTERFACE)
  target_link_libraries(${e_test_app} PUBLIC drishti_c)
  target_compile_definitions(${e_test_app} PUBLIC DRISHTI_BUILD_C_INTERFACE)
endif()

gauze_add_test(
  NAME ${e_test_name}
  COMMAND ${e_test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_MODEL}>"
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
if(NOT DRISHTI_BUILD_HCI OR NOT (TARGET drishti_gltest))
  return()
endif()

set(f_test_name DrishtiDrishtiFaceTest)
set(f_test_app test-drishti-drishti-face)

set(f_sources test-FaceTracker.cpp)
add_executable(${f_test_app} ${f_sources})

set_property(TARGET ${f_test_app} PROPERTY FOLDER "app/tests")
target_link_libraries(${f_test_app} PUBLIC ${drishti_sdk_lib}
  ${OpenCV_LIBS}
  GTest::gtest
  drishtisdk # include drishtisdk for drishti::face::FaceModel serialization
  )

if(DRISHTI_BUILD_OGLES_GPGPU)
  target_link_libraries(${f_test_app} PUBLIC drishti_gltest)
  target_compile_definitions(${f_test_app} PUBLIC DRISHTI_DRISHTI_DO_GPU=1)
endif()

if(DRISHTI_BUILD_C_INTERFACE)
  target_link_libraries(${f_test_app} PUBLIC drishti_c)
  target_compile_definitions(${f_test_app} PUBLIC DRISHTI_BUILD_C_INTERFACE)
endif()

gauze_add_test(
  NAME ${f_test_name}
  COMMAND ${f_test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR_MEAN}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>"
  "0"
  )

if(WIN32)
  set_property(
    TEST ${f_test_name}
    PROPERTY
    ENVIRONMENT
    "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
    )
endif()
