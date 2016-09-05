include_directories("${OpenCV_INCLUDE_DIRS}")

if(ANDROID)
  add_definitions(-DHALF_ENABLE_CPP11_CMATH=0)
endif()

# Disable Clang specific error
message("CMAKE_CXX_COMPILER_ID: ${CMAKE_CXX_COMPILER_ID}")
message("=======")

#### opencv_contrib stuff : tracking, etc
sugar_include(${CMAKE_SOURCE_DIR}/src/3rdparty)

set(LIB_TYPE STATIC)

### XGBOOST ### (SUBMODULE)

option(DRISHTI_USE_LOCAL_XGBOOST "Use local xgboost" ON)
if(DRISHTI_USE_LOCAL_XGBOOST)
  # Note: Use either XGBOOST
  option(SKIP_INSTALL "Avoid installation of submodule libraries" ON)
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost")
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost/src")
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost)
  set(XGBOOST_LIB xgboost)
  unset(SKIP_INSTALL CACHE)
else()
  message( FATAL_ERROR "XGBoost package must export learner-inl.hpp header" )
  # TODO: missing learner-inl.hpp
  # ...my_file.cpp:27:10: fatal error: 'src/learner/learner-inl.hpp' file not found
  hunter_add_package(xgboost)
  find_package(xgboost CONFIG REQUIRED)
  set(XGBOOST_LIB xgboost::xgboost)
endif()

#drishti_print_env_var()
#return()

if(DRISHTI_BUILD_OGLES_GPGPU)

  # Support local submodule use or external package:
  option(DRISHTI_USE_LOCAL_OGLES_GPGPU "Use local ogles_gpgpu" OFF)

  if(DRISHTI_USE_LOCAL_OGLES_GPGPU)
    add_definitions(-DUSE_LOCAL_OGLES_GPGPU=1)
    option(SKIP_INSTALL "Avoid installation of submodule libraries"  ON)
    option(OGLES_GPGPU_BUILD_EXAMPLES "Don't build examples" OFF)
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu/ogles_gpgpu")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu/ogles_gpgpu/common")
    add_subdirectory( "${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu" )
    set(OGLES_GPGPU_LIB ogles_gpgpu)
    unset(SKIP_INSTALL CACHE)
  else()
    add_definitions(-DUSE_LOCAL_OGLES_GPGPU=0)
    hunter_add_package(ogles_gpgpu)
    find_package(ogles_gpgpu CONFIG REQUIRED)
    # TODO: Fix package includes, shouldn't need these
    include_directories("${OGLES_GPGPU_ROOT}/include/ogles_gpgpu")
    include_directories("${OGLES_GPGPU_ROOT}/include/ogles_gpgpu/common")
    set(OGLES_GPGPU_LIB ogles_gpgpu::ogles_gpgpu)
  endif()

endif()

#drishti_print_env_var()

option(DRISHTI_BUILD_EOS "EOS 2D-3D fitting" ON)
if(DRISHTI_BUILD_EOS)
  ## Header only eos lib for 3D-to-2D model fitting
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/eos/include")
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/eos/3rdparty/glm-0.9.7.0") # TODO: hunter package
endif()

if(DRISHTI_BUILD_DEST)
  option(DRISHTI_USE_LOCAL_DEST "Use local dest" OFF)

  if(DRISHTI_USE_LOCAL_DEST)
    add_definitions(-DUSE_LOCAL_DEST=1)
    set(DEST_BUILD_EXAMPLES OFF)
    set(DEST_WITH_OPENCV OFF)
    option(SKIP_INSTALL "Avoid installation of submodule libraries" ON)
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/dest")
    add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/dest)
    set(DEST_LIB dest)
    unset(SKIP_INSTALL CACHE)
    set_property(TARGET dest PROPERTY FOLDER "libs/3rdparty")
    set(DEST_LIB dest)
  else()
    add_definitions(-DUSE_LOCAL_DEST=0)
    hunter_add_package(dest)
    find_package(dest CONFIG REQUIRED)
    set(DEST_LIB dest::dest)
  endif()
endif()

#set(BUILD_SHARED_LIBS ${BUILD_SHARED_LIBS_TOP})

## 3rd party boost pba
include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty/pba)
add_library(boost-pba STATIC ${BOOST_PBA_SRCS})

include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty)

set(DRISHTI_SDK_3RDPARTY_SRC_LIBS
  ${XGBOOST_LIB}
  ${OGLES_GPGPU_LIB}
  ${DEST_LIB}
  boost-pba
  )

## After finding all these create our master dependency list ##
set(DRISHTI_SDK_3RDPARTY_LIBS
  dlib::dlib
  ${CMATIO_LIB}
  ${DRISHTI_SDK_BOOST_LIBS}
  cereal::cereal
  ${OpenCV_LIBS}
  Eigen::eigen
  ${DRISHTI_SDK_3RDPARTY_SRC_LIBS}
  spdlog::spdlog
  )

if(cvmatio_FOUND)
  list(APPEND DRISHTI_SDK_3RDPARTY_LIBS cvmatio::cvmatio)
endif()

# Add the sub-libraries
add_subdirectory(lib)

if(DRISHTI_BUILD_QT)
  # Some background: https://github.com/ruslo/hunter/issues/253
  hunter_add_package(Qt COMPONENTS qtmultimedia qtdeclarative)
endif()

# Include *nix style console apps (some will run on iOS and Android)
add_subdirectory(app)

# (Optional) build unit tests
if(DRISHTI_BUILD_TESTS)
  add_subdirectory(tests)
endif()

