function(drishti_android_copy_files TARGET LOCAL_FILES REMOTE_DIRECTORY)

  if(HUNTER_ENABLED)
    hunter_add_package(Android-SDK)
    set(ADB_COMMAND "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
  else()
    set(ADB_COMMAND "adb")
  endif()
  
  set(VALID_LOCAL_FILES "")
  foreach(local_file ${LOCAL_FILES})
    if(EXISTS "${local_file}")
      message("EXISTS: ${local_file} -> ${REMOTE_DIRECTORY}")
      list(APPEND VALID_LOCAL_FILES "${local_file}")

      # NOTE: Currently running adb push one at a time...
      # to be replaced by single adb push file1 file2 .. filen ${REMOTE_DIRECTORY}
      # pending correct syntax (see below, escape issues, etc)
      add_custom_command(
        TARGET ${TARGET}
        POST_BUILD # Make sure this runs before the ctest step (POST_BUILD OK?)
        COMMAND ${ADB_COMMAND} shell mkdir -p "${REMOTE_DIRECTORY}/"
        COMMAND ${ADB_COMMAND} push "${local_file}" "${REMOTE_DIRECTORY}/"
        )
      
    endif()
  endforeach()

  # list(LENGTH VALID_LOCAL_FILES VALID_LOCAL_FILE_COUNT)
  # message("VALID_LOCAL_FILE_COUNT: ${VALID_LOCAL_FILE_COUNT}")

  # if(${VALID_LOCAL_FILE_COUNT})
  #   string(REPLACE ";" " " VALID_LOCAL_FILES_STRING "${VALID_LOCAL_FILES}") # handle spaces?
  #   message("adb push ${VALID_LOCAL_FILES_STRING} ${REMOTE_DIRECTORY}") 
  #   add_custom_command(
  #     TARGET ${TARGET}
  #     POST_BUILD 
  #     COMMAND ${ADB_COMMAND} push ${VALID_LOCAL_FILES_STRING} "${REMOTE_DIRECTORY}"
  #     VERBATIM
  #     )
  # endif()
  
endfunction()