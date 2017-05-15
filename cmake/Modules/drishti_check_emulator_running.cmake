function(drishti_check_emulator_running device_running_result)
  if("${ANDROID-SDK_ROOT}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()
  if("${DRISHTI_ANDROID_ADB}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()

  set(cmd ${DRISHTI_ANDROID_ADB} devices)
  execute_process(
      COMMAND ${cmd}
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
  )

  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed: ${cmd} (${result}, ${output}, ${error})")
  endif()

  if("${DRISHTI_DEVICE_PORT}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()

  string(REPLACE "\n" ";" output "${output}")
  string(REPLACE "\t" " " output "${output}")
  foreach(x ${output})
    string(REGEX MATCH "^emulator-${DRISHTI_DEVICE_PORT}[ ]+device$" match "${x}")
    if(NOT "${match}" STREQUAL "")
      set(${device_running_result} TRUE PARENT_SCOPE)
      return()
    endif()
  endforeach()

  set(${device_running_result} FALSE PARENT_SCOPE)
endfunction()
