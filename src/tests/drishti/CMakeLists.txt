set(test_name DrishtiTest)
set(test_app test-drishti)

set(DRISHTI_UT_SOURCES
  ${DRISHTI_CORE_UT}
  ${DRISHTI_GEOMETRY_UT}
  ${DRISHTI_ML_UT}  
  ${DRISHTI_ACF_UT}
  ${DRISHTI_EYE_UT}
  ${DRISHTI_FACE_UT}
  ${DRISHTI_HCI_UT}
  ${DRISHTI_DRISHTI_UT} # public API layer
  )

# Use static lib to avoid dynamic frameworks in internal tests
if(IOS)
  set(drishti_sdk_lib drishti_static)
else()
  set(drishti_sdk_lib drishti)
endif()

add_executable(${test_app} test-drishti.cpp ${DRISHTI_UT_SOURCES})
target_link_libraries(${test_app} PUBLIC
  GTest::gtest
  ${OpenCV_LIBS}
  ${drishti_sdk_lib}  
  drishtisdk
  )

if(${DRISHTI_DO_GPU_TESTING})
  target_link_libraries(${test_app} PUBLIC aglet::aglet)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_DO_GPU_TESTING=1)
endif()

if(DRISHTI_BUILD_C_INTERFACE)
  target_link_libraries(${test_app} PUBLIC drishti_c)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_BUILD_C_INTERFACE=1)
endif()

target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>")
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

if(WIN32)
  set_property(
    TEST ${e_test_name}
    PROPERTY
    ENVIRONMENT
    "PATH=$<TARGET_FILE_DIR:drishti>;$ENV{PATH}"
    )
endif()


gauze_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_DETECTOR_MEAN}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_ASSETS_EYE_MODEL_REGRESSOR}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_MODEL}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_EYE_MODEL_PRIVATE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>"
  "$<GAUZE_RESOURCE_FILE:${DRISHTI_FACES_FACE_IMAGE}>" # TODO: model
  "."
  )

#####

