#
# This file manages integration tests (post-install package config libraries):
#
# Note that the user must specify the following definintions:
#
# * drishti_DIR
# * opencv_DIR
# * PNG_DIR
# * ZLIB_DIR

set(test_name DrishtiIntegrationTest)
set(test_app test-drishti-intergration)

# hunter_add_package(drishti) # TODO
add_executable(${test_app} drishti-example-eye.cpp)

# Optionally forward drishti_DIR argument for find_package()
if(IOS)

  # CMake doesn't like multiple "-F" options, as the second one is automatically
  # stripped, leading to compiler errors:
  # clang: error: cannot specify -o when generating multiple output files  
  
  # target_compile_options(drishti-example-eye
  #   PUBLIC
  #   "-F" "${drishti_DIR}"
  #   "-F" "${OpenCV_DIR}"
  #   )

  # workaround :point_up:
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -F ${drishti_DIR} -F ${OpenCV_DIR}")

  target_link_libraries(${test_app}
    PUBLIC
    "-F ${OpenCV_DIR} -framework opencv2"
    "-F ${drishti_DIR} -framework drishti"
    GTest::gtest
    )

  set_target_properties(
    ${test_app}
    PROPERTIES
    XCODE_ATTRIBUTE_PRODUCT_NAME "${test_app}"
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad 
    XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/Frameworks"
    )

  # See: http://stackoverflow.com/a/39042153
  add_custom_command(
    TARGET ${test_app}
    PRE_BUILD
    COMMAND
    "${CMAKE_COMMAND}" -E copy_directory
    "${drishti_DIR}/drishti.framework"
    "$<TARGET_FILE_DIR:${test_app}>/Frameworks/drishti.framework"
    )

else()

  find_package(drishti CONFIG REQUIRED)
  find_package(OpenCV REQUIRED)
  message(":::::::: :::: :: : OpenCV_DIR == ${OpenCV_DIR}")
  
  target_link_libraries(${test_app}
    PUBLIC
    drishti::drishti
    ${OpenCV_LIBS}
    )
  
endif()

#############
### TEST ####
#############

include(drishti_set_unit_test_assets)
drishti_set_unit_test_assets(
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

set(DRISHTI_TEST_EYE_IMAGE "${assets_dir}/images/2318-eye.png")
set(DRISHTI_TEST_EYE_JSON "${assets_dir}/images/2318-eye.json")
message("DRISHTI_TEST_EYE_MODEL_PRIVATE ${DRISHTI_TEST_EYE_MODEL_PRIVATE}")

include(drishti_add_test)
drishti_add_test(
  NAME ${test_name}
  COMMAND ${test_app}
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_EYE_MODEL}>"
  "$<DRISHTI_RESOURCE_FILE:${DRISHTI_TEST_EYE_IMAGE}>"
  "1"  
  "."
  )