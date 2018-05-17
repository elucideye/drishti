hunter_config(
  xgboost
  VERSION 0.40-p10 # v0.7.0 introduces significant API changes
  CMAKE_ARGS XGBOOST_USE_HALF=ON XGBOOST_USE_CEREAL=ON XGBOOST_DO_LEAN=${DRISHTI_BUILD_MIN_SIZE}
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
  set(dlib_version 19.2-p2)
  set(nlohmann_json_version 2.1.1-p1)
else()
  set(dlib_version ${HUNTER_dlib_VERSION})
  set(nlohmann_json_version ${HUNTER_nlohmann_json_VERSION})  
endif()

hunter_config(dlib VERSION ${dlib_version} CMAKE_ARGS ${dlib_cmake_args})
hunter_config(nlohmann_json VERSION ${nlohmann_json_version})


option(DRISHTI_ACF_AS_SUBMODULE "Use drishti acf as submodule" ON)
set(acf_cmake_args
  ACF_BUILD_TESTS=OFF 
  ACF_BUILD_EXAMPLES=OFF
  ACF_SERIALIZE_WITH_CVMATIO=${DRISHTI_SERIALIZE_WITH_CVMATIO}
  ACF_SERIALIZE_WITH_CEREAL=ON
  ACF_BUILD_OGLES_GPGPU=${DRISHTI_BUILD_OGLES_GPGPU}
  ACF_KEEPS_SOURCES=1
)

if(DRISHTI_ACF_AS_SUBMODULE)
  hunter_config(acf GIT_SUBMODULE "src/3rdparty/acf" CMAKE_ARGS ${acf_cmake_args})
else()
  hunter_config(acf VERSION ${HUNTER_acf_VERSION} CMAKE_ARGS ${acf_cmake_args})
endif()

set(eigen_cmake_args
  BUILD_TESTING=OFF
  HUNTER_INSTALL_LICENSE_FILES=COPYING.MPL2
  CMAKE_Fortran_COMPILER=OFF
)
hunter_config(Eigen VERSION ${HUNTER_Eigen_VERSION} CMAKE_ARGS ${eigen_cmake_args})

#
# Provide a functional minimal config for the opencv_contrib module:
#

option(DRISHTI_BUILD_OPENCV_EXTRA "Build opencv_contrib modules" OFF) 

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

set(opencv_cmake_args

  OPENCV_WITH_EXTRA_MODULES=${DRISHTI_BUILD_OPENCV_EXTRA} # <===== opencv_contrib    
  
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

  # include/jasper/jas_math.h:184:15: error: 'SIZE_MAX' undeclared (first use in this function)    
  BUILD_JASER=OFF
  WITH_JASPER=OFF
  
  # This stuff will build shared libraries. Build with PIC required for dependencies.
  BUILD_opencv_java=OFF
  BUILD_opencv_python2=OFF
  BUILD_opencv_python3=OFF
  # There is not a CUDA package so need to stop OpenCV from searching for it, otherwise
  #  it might pick up the host version
  WITH_CUDA=OFF
  WITH_CUFFT=OFF
  WITH_EIGEN=OFF

  WITH_PROTOBUF=OFF    # avoid protobuf errors
  BUILD_PROTOBUF=OFF   #  -/-
  BUILD_opencv_dnn=OFF #  -/-
  BUILD_LIBPROTOBUF_FROM_SOURCES=NO

  BUILD_opencv_imgproc=ON   # required for flow
  BUILD_opencv_optflow=ON   # optflow
  BUILD_opencv_plot=ON      # tracking -> plot    
  BUILD_opencv_tracking=ON  # tracking
  BUILD_opencv_video=ON     # tracking -> video
  BUILD_opencv_ximgproc=ON  # optflow -> ximgproc

  # Disable unused opencv_contrib modules (when OPENCV_WITH_EXTRA_MODULES == YES)
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

hunter_config(OpenCV VERSION ${HUNTER_OpenCV_VERSION} CMAKE_ARGS ${opencv_cmake_args})
