message(">:>:>:>:>:>:>:>:>:>:>:>:>:>:>:>:>: ${CMAKE_MODULE_PATH} <:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:<:")

message("-- config.cmake --")
message("  MSVC: ${MSVC}")
message("  APPLE: ${APPLE}")
message("  ANDROID: ${ANDROID}")
message("  IOS: ${IOS}")
message("  is_linux: ${is_linux}")
message("  XCODE: ${XCODE}")

### OpenCV
if(ANDROID)
  message("ANDROID ========================================================================")
  include(drishti_set_opencv_cmake_args_android)
  drishti_set_opencv_cmake_args_android()
elseif(IOS)
  message("IOS ============================================================================")
  include(drishti_set_opencv_cmake_args_ios)
  drishti_set_opencv_cmake_args_ios()
elseif(APPLE)
  message("APPLE ==========================================================================")
  include(drishti_set_opencv_cmake_args_osx)
  drishti_set_opencv_cmake_args_osx()
elseif(${is_linux})
  message("is_linux =======================================================================")
  include(drishti_set_opencv_cmake_args_nix)
  drishti_set_opencv_cmake_args_nix()
elseif(MSVC)
  message("MSVC ===========================================================================")
  include(drishti_set_opencv_cmake_args_windows)
  drishti_set_opencv_cmake_args_windows()
endif()

list(APPEND OPENCV_CMAKE_ARGS
  BUILD_opencv_world=ON
  BUILD_opencv_ts=OFF
  BUILD_opencv_python2=OFF
  BUILD_opencv_shape=OFF
  BUILD_opencv_superres=OFF
  HAVE_OPENCL=OFF
  WITH_OPENCL=OFF
  BUILD_EIGEN=OFF  ### for convenient linking
)

hunter_config(OpenCV VERSION 3.0.0-p6 CMAKE_ARGS "${OPENCV_CMAKE_ARGS}")

### Boost
hunter_config(Boost VERSION 1.58.0-p1 CMAKE_ARGS IOSTREAMS_NO_BZIP2=1 )

hunter_config(xgboost VERSION 0.40-p4)
hunter_config(PNG VERSION 1.6.16-p4)
hunter_config(TIFF VERSION 4.0.2-p3)
hunter_config(cereal VERSION 1.1.2-p5)
hunter_config(cvmatio VERSION 1.0.27-p3)
hunter_config(dlib VERSION 18.14-p1)
hunter_config(Eigen VERSION 3.2.4-p0)
hunter_config(spdlog VERSION 1.0.0-p0)
hunter_config(Qt VERSION 5.5.1-cvpixelbuffer-2-p3)

# From hunter/cmake/configs/default.cmake
if(ANDROID)
  string(COMPARE EQUAL "${ANDROID_NATIVE_API_LEVEL}" "" _is_empty)
  if(_is_empty)
    hunter_user_error("ANDROID_NATIVE_API_LEVEL is empty")
  endif()

  string(COMPARE EQUAL "${ANDROID_NATIVE_API_LEVEL}" "21" _is_api_21)
  string(COMPARE EQUAL "${ANDROID_NATIVE_API_LEVEL}" "19" _is_api_19)
  string(COMPARE EQUAL "${ANDROID_NATIVE_API_LEVEL}" "16" _is_api_16)

  if(_is_api_21)
    hunter_config(Android-Google-APIs VERSION 21)
    hunter_config(Android-Google-APIs-Intel-x86-Atom-System-Image VERSION 21)
    hunter_config(Android-Intel-x86-Atom-System-Image VERSION 21)
    hunter_config(Android-SDK-Platform VERSION 21_r02)
    hunter_config(Sources-for-Android-SDK VERSION 21)
  elseif(_is_api_19)
    hunter_config(Android-Google-APIs VERSION 19)
    hunter_config(Android-Intel-x86-Atom-System-Image VERSION 19)
    hunter_config(Android-SDK-Platform VERSION 19_r04)
    hunter_config(Sources-for-Android-SDK VERSION 19)

    # There is no package for API 19, use version 21
    # (simplify build of Android-SDK package)
    hunter_config(Android-Google-APIs-Intel-x86-Atom-System-Image VERSION 21)
  elseif(_is_api_16)
    hunter_config(Android-Google-APIs VERSION 16)
    hunter_config(Android-Intel-x86-Atom-System-Image VERSION 16)
    hunter_config(Android-SDK-Platform VERSION 16_r05)
    hunter_config(Sources-for-Android-SDK VERSION 16)

    # There is no package for API 16, use version 21
    # (simplify build of Android-SDK package)
    hunter_config(Android-Google-APIs-Intel-x86-Atom-System-Image VERSION 21)
  else()
    hunter_user_error(
        "Android API (ANDROID_NATIVE_API_LEVEL)"
        " Expected: `21`, `19`, `16`"
        " Got: `${ANDROID_NATIVE_API_LEVEL}`"
    )
  endif()
endif()
