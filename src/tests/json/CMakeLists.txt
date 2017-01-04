set(test_name NlohmannJsonTest)
set(test_app test-nlohmann-json)
if(IOS)
  set(test_app_type "MACOSX_BUNDLE")
endif()

### packages ###
hunter_add_package(nlohmann-json)
find_package(nlohmann-json REQUIRED)

add_executable(${test_app} ${test_app_type} test-json.cpp)
target_link_libraries(${test_app}  nlohmann-json::nlohmann-json GTest::gtest)
set_property(TARGET ${test_app} PROPERTY FOLDER "app/tests")

enable_testing()

set(DRISHTI_JSON_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sample.json")

if(ANDROID OR IOS)

  get_filename_component(DRISHTI_JSON_TEST_FILE_ "${DRISHTI_JSON_TEST_FILE}" NAME)

  if(EXISTS "${DRISHTI_JSON_TEST_FILE}")
    message("file exists: ${DRISHTI_JSON_TEST_FILE}")
  endif()
  
  if(ANDROID)
    drishti_android_add_test(
      NAME ${test_name}
      COMMAND ${test_app}
      "${DRISHTI_JSON_TEST_FILE_}"
      DEPENDENCIES
      "${DRISHTI_JSON_TEST_FILE}"      
      CLEAN
      )
    
  elseif(IOS)

    set(test_bundle_id "com.example.tests.nlohmann.json") 
    set_target_properties(
      ${test_app}
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
      "${test_bundle_id}"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )

    drishti_ios_add_test(
      NAME ${test_name}
      BUNDLE_ID "${test_bundle_id}"      
      COMMAND ${test_app}
      "~/Documents/${DRISHTI_JSON_TEST_FILE_}"      
      DEPENDENCIES
      "${DRISHTI_JSON_TEST_FILE}"      
      #CLEAN
      )
    
  endif()
  
else()
  
  add_test(NAME ${test_name} COMMAND ${test_app}
    "${DRISHTI_JSON_TEST_FILE}"
    )
  
endif()
