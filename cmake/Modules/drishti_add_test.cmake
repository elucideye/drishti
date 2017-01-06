cmake_minimum_required(VERSION 3.1)

include(CMakeParseArguments) # cmake_parse_arguments

set(DRISHTI_ADD_TEST_SELF_DIR "${CMAKE_CURRENT_LIST_DIR}")

function(drishti_add_test)
  set(optional "")
  set(one NAME)
  set(multiple COMMAND)

  # * x_NAME
  # * x_COMMAND
  cmake_parse_arguments(x "${optional}" "${one}" "${multiple}" "${ARGV}")

  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)
  if(has_unparsed)
    message(FATAL_ERROR "Unparsed arguments: ${x_UNPARSED_ARGUMENTS}")
  endif()

  if("${x_NAME}" STREQUAL "")
    message(FATAL_ERROR "NAME required")
  endif()

  if("${x_COMMAND}" STREQUAL "")
    message(FATAL_ERROR "COMMAND required")
  endif()

  list(GET x_COMMAND 0 APP_TARGET)
  if(NOT TARGET "${APP_TARGET}")
    message(
        FATAL_ERROR
        "Expected executable target as first argument, but got: ${APP_TARGET}"
    )
  endif()

  list(REMOVE_AT x_COMMAND 0)
  set(APP_ARGUMENTS ${x_COMMAND})

  set(
      script_path
      "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/DrishtiTest/${x_NAME}.cmake"
  )

  if(ANDROID)
    hunter_add_package(Android-SDK)
    set(ADB_COMMAND "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")

    set(
        DRISHTI_ANDROID_DEVICE_TESTING_ROOT
        "/data/local/tmp"
        CACHE
        STRING
        "Android device testing root directory"
    )

    set(TESTING_DIR "${DRISHTI_ANDROID_DEVICE_TESTING_ROOT}/${PROJECT_NAME}")

    # Use:
    # * ADB_COMMAND
    # * APP_TARGET
    # * APP_ARGUMENTS
    # * TESTING_DIR
    configure_file(
        "${DRISHTI_ADD_TEST_SELF_DIR}/templates/AndroidTest.cmake.in"
        "${script_path}"
        @ONLY
    )

    add_test(
        NAME "${x_NAME}"
        COMMAND
            "${CMAKE_COMMAND}"
            "-DAPP_SOURCE=$<TARGET_FILE:${APP_TARGET}>"
            -P
            "${script_path}"
    )
  elseif(IOS)
    message(FATAL_ERROR "TODO")
  else()
    add_test(NAME "${x_NAME}" COMMAND ${APP_TARGET} ${x_COMMAND})
  endif()
endfunction()
