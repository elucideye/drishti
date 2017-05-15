include(drishti_check_emulator_running)
include(drishti_create_avd)
include(drishti_run_start_script)

function(drishti_start_android_emulator)
  hunter_add_package(Android-SDK) # ANDROID-SDK_ROOT

  set(DRISHTI_DEVICE_PORT 5678)
  set(DRISHTI_DEVICE_NAME "Drishti_android-${CMAKE_SYSTEM_VERSION}_${CMAKE_ANDROID_ARCH_ABI}")
  set(DRISHTI_ANDROID_ADB "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
  set(DRISHTI_ANDROID_EMULATOR "${ANDROID-SDK_ROOT}/android-sdk/tools/emulator")

  drishti_check_emulator_running(result)
  if(result)
    message("Device already is running")
    return()
  endif()

  drishti_create_avd()
  drishti_run_start_script()

  drishti_check_emulator_running(result)
  if(NOT result)
    message(FATAL_ERROR "Device is not running")
  endif()
endfunction()
