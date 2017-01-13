function(drishti_android_copy_files TARGET LOCAL_FILES REMOTE_DIRECTORY)

  # if(HUNTER_ENABLED)
  #   hunter_add_package(Android-SDK)
  #   set(ADB_COMMAND "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
  # else()
  #   set(ADB_COMMAND "adb")
  # endif()

  # Avoid installation of Android-SDK on Travis
  set(ADB_COMMAND "adb")    
  
  foreach(local_file ${LOCAL_FILES})
    if(EXISTS "${local_file}")
      message("EXISTS: ${local_file} -> ${REMOTE_DIRECTORY}")
      add_custom_command(
        TARGET ${TARGET}
        POST_BUILD # Make sure this runs before the ctest step (POST_BUILD OK?)
        COMMAND ${ADB_COMMAND} shell mkdir -p "${REMOTE_DIRECTORY}/"
        COMMAND ${ADB_COMMAND} push "${local_file}" "${REMOTE_DIRECTORY}/"
        )
      
    endif()
  endforeach()
  
endfunction()