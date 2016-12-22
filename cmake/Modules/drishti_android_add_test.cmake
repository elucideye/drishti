# Lineage: https://github.com/hunter-packages/android-apk/blob/master/AndroidApk.cmake
# Local copy of android_add_test() to extend functionality
# Changes/proposals to be submitted back to source

##################################################
## FUNCTION: android_add_test
##
## Run test on device (similar to add_test)
##
## @param NAME
##   Name of the test
## @param COMMAND
##   Command to test
##################################################

function(drishti_android_add_test)

  # if(HUNTER_ENABLED)
  #   hunter_add_package(Android-SDK)
  #   set(ADB_COMMAND "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
  # else()
  #   set(ADB_COMMAND "adb")
  # endif()

  # Avoid installation of Android-SDK on Travis
  set(ADB_COMMAND "adb")  

  # Directory on device for storing applications
  # FIXME: user control
  set(ANDROID_APK_APP_DESTINATION "/data/local/tmp/AndroidApk")
  
  # Introduce:
  # * x_NAME
  # * x_COMMAND
  cmake_parse_arguments(x "" "NAME" "COMMAND" ${ARGV})
  string(COMPARE NOTEQUAL "${x_UNPARSED_ARGUMENTS}" "" has_unparsed)

  if(has_unparsed)
    message(FATAL_ERROR "Unparsed: ${x_UNPARSED_ARGUMENTS}")
  endif()

  list(GET x_COMMAND 0 app_target)
  if(NOT TARGET "${app_target}")
    message(
        FATAL_ERROR
        "Expected executable target as first argument, but got: ${app_target}"
    )
  endif()

  set(APP_DESTINATION_DIR "${ANDROID_APK_APP_DESTINATION}")
  set(APP_DESTINATION_DIR "${APP_DESTINATION_DIR}/${PROJECT_NAME}/AndroidTest")
  set(APP_DESTINATION_DIR "${APP_DESTINATION_DIR}/${x_NAME}")
  set(APP_DESTINATION "${APP_DESTINATION_DIR}/${app_target}")

  # Set variable ${NAME}_INSTALL_PATH variable at parent scope:
  set("${x_NAME}_INSTALL_PATH" "${APP_DESTINATION_DIR}" PARENT_SCOPE)
  message("${x_NAME}_INSTALL_PATH" "${APP_DESTINATION_DIR}")

  set(
      script_loc
      "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/AndroidTest/${x_NAME}.cmake"
  )

  list(REMOVE_AT x_COMMAND 0)
  set(APP_ARGUMENTS ${x_COMMAND})
  # Use:
  # * ADB_COMMAND
  # * APP_ARGUMENTS
  # * APP_DESTINATION
  configure_file(
    #"${_ANDROID_APK_THIS_DIRECTORY}/templates/AndroidTest.cmake.in"
      "${CMAKE_SOURCE_DIR}/cmake/templates/AndroidTest.cmake.in" # local copy
      "${script_loc}"
      @ONLY
  )

  add_test(
      NAME "${x_NAME}"
      COMMAND
          "${CMAKE_COMMAND}"
          "-DAPP_SOURCE=$<TARGET_FILE:${app_target}>"
          -P
          "${script_loc}"
  )
endfunction()