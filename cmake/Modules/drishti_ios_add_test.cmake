# Lineage: https://github.com/hunter-packages/android-apk/blob/master/AndroidApk.cmake
# Local copy of android_add_test() to extend functionality
# Changes/proposals to be submitted back to source

####################################################################
## FUNCTION: ios_add_test
##
## Run test on device (similar to add_test)
##
## @param NAME
##   Name of the test
## @param COMMAND
##   Command to test
## @param DEPENDENCIES
##   Files to be copied to the same directory as the application
## @param CLEAN
##   Remove installed files after the test
##
## This will set LD_LIBRARY_PATH to the installation
## path of the executable, so that dynamic linking
## with shared libraries copied to the same directory
## will succeed.
####################################################################

function(drishti_ios_add_test)

  set(IOS_DEPLOY_COMMAND "ios-deploy")  

  # Introduce:
  # * x_NAME
  # * x_COMMAND
  # * x_DEPENDENCIES
  # * x_CLEAN  
  set(options CLEAN)
  set(oneValueArgs NAME)
  set(multiValueArgs COMMAND DEPENDENCIES)
  cmake_parse_arguments(x "${options}" "${oneValueArgs}" "${multiValueArgs}" ${ARGV})
  
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

  # Query the XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
  get_target_property(x_BUNDLE_ID ${app_target} XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER)

  set(
      script_loc
      "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/iOSTest/${x_NAME}.cmake"
  )

  list(REMOVE_AT x_COMMAND 0)
  set(APP_ARGUMENTS ${x_COMMAND})
  # Use:
  # * DEPLOY_COMMAND
  # * APP_ARGUMENTS
  # * APP_DESTINATION
  configure_file(
      "${CMAKE_SOURCE_DIR}/cmake/templates/iOSTest.cmake.in" # local copy
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
