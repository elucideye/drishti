function(drishti_create_avd)
  if("${ANDROID-SDK_ROOT}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()

  if("${DRISHTI_DEVICE_NAME}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()

  set(android_tool "${ANDROID-SDK_ROOT}/android-sdk/tools/android")

  set(cmd ${android_tool} list avd)
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

  string(REGEX MATCH "\n    Name: ${DRISHTI_DEVICE_NAME}\n" avd_found "${output}")
  if(NOT "${avd_found}" STREQUAL "")
    message("AVD with name '${DRISHTI_DEVICE_NAME}' already exist")
    return()
  endif()

  message("Creating AVD with name '${DRISHTI_DEVICE_NAME}'")
  set(
      cmd
      ${android_tool}
      create
      avd
      --name "${DRISHTI_DEVICE_NAME}"
      --target "android-${CMAKE_SYSTEM_VERSION}"
      --abi "${CMAKE_ANDROID_ARCH_ABI}"
  )
  execute_process(
      COMMAND ${CMAKE_COMMAND} -E echo no
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
endfunction()
