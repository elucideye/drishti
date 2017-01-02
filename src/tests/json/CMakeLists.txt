set(DRISHTI_TEST_NAME NlohmannJsonTest)
set(DRISHTI_TEST_APP test-nlohmann-json)
if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

### packages ###
hunter_add_package(nlohmann-json)
find_package(nlohmann-json REQUIRED)

add_executable(${DRISHTI_TEST_APP} ${TEST_APP_TYPE} test-json.cpp)
target_link_libraries(${DRISHTI_TEST_APP}  nlohmann-json::nlohmann-json GTest::gtest)
set_property(TARGET ${DRISHTI_TEST_APP} PROPERTY FOLDER "app/tests")
if(DRISHTI_HAVE_REGEX_H)
  target_compile_definitions(${DRISHTI_TEST_APP} PUBLIC DRISHTI_HAVE_REGEX_H)
endif()

enable_testing()

set(DRISHTI_JSON_TEST_FILE "${CMAKE_CURRENT_SOURCE_DIR}/sample.json")

if(ANDROID OR IOS)

  get_filename_component(DRISHTI_JSON_TEST_FILE_ "${DRISHTI_JSON_TEST_FILE}" NAME)

  if(EXISTS "${DRISHTI_JSON_TEST_FILE}")
    message("file exists: ${DRISHTI_JSON_TEST_FILE}")
  endif()
  
  if(ANDROID)
    drishti_android_add_test(
      NAME ${DRISHTI_TEST_NAME}
      COMMAND ${DRISHTI_TEST_APP}
      "${DRISHTI_JSON_TEST_FILE_}"
      DEPENDENCIES
      "${DRISHTI_JSON_TEST_FILE}"      
      CLEAN
      )
    
  elseif(IOS)

    set(DRISHTI_TEST_BUNDLE_IDENTIFIER "com.elucideye.nlohmann.json") 
    set_target_properties(
      ${DRISHTI_TEST_APP}
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST
      "${CMAKE_SOURCE_DIR}/cmake/templates/Info.plist.in"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
      "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )

    drishti_ios_add_test(
      NAME ${DRISHTI_TEST_NAME}
      BUNDLE_ID "${DRISHTI_TEST_BUNDLE_IDENTIFIER}"      
      COMMAND ${DRISHTI_TEST_APP}
      "~/Documents/${DRISHTI_JSON_TEST_FILE_}"      
      DEPENDENCIES
      "${DRISHTI_JSON_TEST_FILE}"      
      #CLEAN
      )
    
  endif()
  
else()
  
  add_test(NAME ${DRISHTI_TEST_NAME} COMMAND ${DRISHTI_TEST_APP})
  
endif()
