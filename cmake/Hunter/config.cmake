hunter_config(
  xgboost
  VERSION 0.40-p10 # v0.7.0 introduces significant API changes
  CMAKE_ARGS XGBOOST_USE_HALF=ON XGBOOST_USE_CEREAL=ON XGBOOST_DO_LEAN=ON
  )

set(dlib_cmake_args
  DLIB_HEADER_ONLY=OFF  #all previous builds were header on, so that is the default
  DLIB_ENABLE_ASSERTS=OFF #must be set on/off or debug/release build will differ and config will not match one
  DLIB_NO_GUI_SUPPORT=ON
  DLIB_ISO_CPP_ONLY=OFF # needed for directory navigation code (loading training data)
  DLIB_JPEG_SUPPORT=OFF  # https://github.com/hunter-packages/dlib/blob/eb79843227d0be45e1efa68ef9cc6cc187338c8e/dlib/CMakeLists.txt#L422-L432
  DLIB_LINK_WITH_SQLITE3=OFF
  DLIB_USE_BLAS=OFF
  DLIB_USE_LAPACK=OFF
  DLIB_USE_CUDA=OFF
  DLIB_PNG_SUPPORT=ON
  DLIB_GIF_SUPPORT=OFF
  DLIB_USE_MKL_FFT=OFF  
  HUNTER_INSTALL_LICENSE_FILES=dlib/LICENSE.txt
)

if(ANDROID)
  # https://travis-ci.org/ingenue/hunter/jobs/287844545
  # Will be fixed in Android NDK 17
  set(dlib_version 19.2-p1)
else()
  set(dlib_version ${HUNTER_dlib_VERSION})
endif()
    
hunter_config(dlib VERSION ${dlib_version} CMAKE_ARGS ${dlib_cmake_args})

set(acf_cmake_args
  ACF_BUILD_TESTS=OFF 
  ACF_BUILD_EXAMPLES=OFF
  ACF_SERIALIZE_WITH_CVMATIO=${DRISHTI_SERIALIZE_WITH_CVMATIO}
  ACF_SERIALIZE_WITH_CEREAL=ON
  ACF_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU}
)

hunter_config(acf VERSION ${HUNTER_acf_VERSION} CMAKE_ARGS ${acf_cmake_args})

#
# Provide a functional minimal config for the opencv_contrib module:
#

option(DRISHTI_BUILD_OPENCV_EXTRA "Build opencv_contrib modules" OFF) 
if (DRISHTI_BUILD_OPENCV_EXTRA)
  string(COMPARE EQUAL "${CMAKE_OSX_SYSROOT}" "iphoneos" _is_ios)

  if(_is_ios)
    set(_ios_args BUILD_WEBP=ON)
  else()
    set(_ios_args "")
  endif()

  if(ANDROID)
    # This feature doesn't work with new CMake 3.7+ toolchains
    set(_android_args ENABLE_PRECOMPILED_HEADERS=OFF)
  else()
    set(_android_args "")
  endif()

  set(OPENCV_CMAKE_ARGS
    BUILD_ANDROID_EXAMPLES=OFF
    BUILD_DOCS=OFF
    BUILD_EXAMPLES=OFF
    BUILD_PERF_TESTS=OFF
    BUILD_TESTS=OFF
    BUILD_opencv_apps=OFF
    INSTALL_PYTHON_EXAMPLES=OFF
    BUILD_WITH_STATIC_CRT=OFF # Fix https://github.com/ruslo/hunter/issues/177
    ${_ios_args}
    ${_android_args}
    # Find packages in Hunter (instead of building from OpenCV sources)
    BUILD_ZLIB=OFF
    BUILD_TIFF=OFF
    BUILD_PNG=OFF
    BUILD_JPEG=OFF
    # This stuff will build shared libraries. Build with PIC required for dependencies.
    BUILD_opencv_java=OFF
    BUILD_opencv_python2=OFF
    BUILD_opencv_python3=OFF
    # There is not a CUDA package so need to stop OpenCV from searching for it, otherwise
    #  it might pick up the host version
    WITH_CUDA=OFF
    WITH_CUFFT=OFF
    WITH_EIGEN=OFF
    OPENCV_WITH_EXTRA_MODULES=YES # <===== opencv_contrib

    BUILD_opencv_dnn=OFF
    BUILD_LIBPROTOBUF_FROM_SOURCES=YES

    BUILD_opencv_imgproc=ON   
    BUILD_opencv_optflow=ON   # optflow
    BUILD_opencv_plot=ON      # tracking -> plot    
    BUILD_opencv_tracking=ON  # tracking
    BUILD_opencv_video=ON     # tracking -> video
    BUILD_opencv_ximgproc=ON  # optflow -> ximgproc
    
    BUILD_opencv_aruco=OFF
    BUILD_opencv_bgsegm=OFF
    BUILD_opencv_bioinspired=OFF
    BUILD_opencv_ccalib=OFF
    BUILD_opencv_cvv=OFF
    BUILD_opencv_datasets=OFF
    BUILD_opencv_dpm=OFF
    BUILD_opencv_face=OFF
    BUILD_opencv_fuzzy=OFF
    BUILD_opencv_hdf=OFF
    BUILD_opencv_line_descriptor=OFF
    BUILD_opencv_reg=OFF
    BUILD_opencv_rgbd=OFF
    BUILD_opencv_saliency=OFF
    BUILD_opencv_sfm=OFF
    BUILD_opencv_stereo=OFF
    BUILD_opencv_structured_light=OFF
    BUILD_opencv_surface_matching=OFF
    BUILD_opencv_text=OFF
    BUILD_opencv_xfeatures2d=OFF
    BUILD_opencv_xobjdetect=OFF
    BUILD_opencv_xphoto=OFF
    )

  hunter_config(OpenCV VERSION ${HUNTER_OpenCV_VERSION} CMAKE_ARGS ${OPENCV_CMAKE_ARGS})

endif()
