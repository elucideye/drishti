set(DRISHTI_START_EMULATOR_SCRIPT "${CMAKE_CURRENT_LIST_DIR}/templates/start-emulator.sh.in")

function(drishti_run_start_script)
  if("${DRISHTI_ANDROID_ADB}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()
  if("${DRISHTI_ANDROID_EMULATOR}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()
  if("${DRISHTI_DEVICE_NAME}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()
  if("${DRISHTI_DEVICE_PORT}" STREQUAL "")
    message(FATAL_ERROR "Internal error")
  endif()

  set(start_script "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/Drishti/start-emulator.sh")
  set(DRISHTI_START_LOG "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/Drishti/start-log.txt")
  set(DRISHTI_START_ERR "${CMAKE_CURRENT_BINARY_DIR}/_3rdParty/Drishti/start-err.txt")

  # Use:
  # * DRISHTI_ANDROID_ADB
  # * DRISHTI_ANDROID_EMULATOR
  # * DRISHTI_DEVICE_NAME
  # * DRISHTI_DEVICE_PORT
  # * DRISHTI_START_ERR
  # * DRISHTI_START_LOG
  # * ANDROID-SDK_ROOT
  configure_file(${DRISHTI_START_EMULATOR_SCRIPT} "${start_script}" @ONLY)

  set(bash_path "/bin/bash")
  if(NOT EXISTS "${bash_path}")
    message(FATAL_ERROR "File not found: '${bash_path}'")
  endif()

  message("Starting emulator")
  set(cmd "${bash_path}" ${start_script})
  execute_process(
      COMMAND ${cmd}
      WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
      RESULT_VARIABLE result
      OUTPUT_VARIABLE output
      ERROR_VARIABLE error
      OUTPUT_STRIP_TRAILING_WHITESPACE
      ERROR_STRIP_TRAILING_WHITESPACE
  )

  if(NOT result EQUAL 0)
    message(FATAL_ERROR "Command failed: ${cmd} (${result}, ${output}, ${error})")
  endif()

  message("Waiting for emulator")
  foreach(x RANGE 15) # wait for 15 minutes
    set(cmd "${DRISHTI_ANDROID_ADB}" -e wait-for-device)
    execute_process(
        COMMAND ${cmd}
        TIMEOUT 60 # exit every minute to read logs
        WORKING_DIRECTORY "${CMAKE_CURRENT_BINARY_DIR}"
        RESULT_VARIABLE result
        OUTPUT_VARIABLE output
        ERROR_VARIABLE error
        OUTPUT_STRIP_TRAILING_WHITESPACE
        ERROR_STRIP_TRAILING_WHITESPACE
    )

    if(result EQUAL 0)
      message("Emulator is ready!")
      return()
    endif()

    file(READ "${DRISHTI_START_LOG}" log)
    file(READ "${DRISHTI_START_ERR}" err)

    message("Current logs:")
    message("--- LOG (stdout) BEGIN ---")
    message("${log}")
    message("--- LOG (stdout) END ---")
    message("--- LOG (stderr) BEGIN ---")
    message("${err}")
    message("--- LOG (stderr) END ---")
  endforeach()

  message(FATAL_ERROR "Emulator start failed (exit by timeout)")
endfunction()
