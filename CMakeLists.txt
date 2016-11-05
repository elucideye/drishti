cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/drishti-upload/cmake/Modules")

include("cmake/HunterGate.cmake")
### Hunter snapshot that will be used ###

# drishti specific hunter cache pacakges:
list(APPEND HUNTER_CACHE_SERVERS "https://github.com/elucideye/hunter-cache")

HunterGate(
  URL "https://github.com/ruslo/hunter/archive/v0.16.23.tar.gz"
  SHA1 "5d73b3dacceb26452ae9267f433341a3d6f7c6c2"  
  FILEPATH "${CMAKE_CURRENT_LIST_DIR}/drishti-upload/config.cmake"
  )

string(COMPARE NOTEQUAL "$ENV{TRAVIS_TAG}" "" travis_deploy)
string(COMPARE EQUAL "$ENV{APPVEYOR_REPO_TAG}" "true" appveyor_deploy)

if(travis_deploy)
  set(version "$ENV{TRAVIS_TAG}")
elseif(appveyor_deploy)
  set(version "$ENV{APPVEYOR_REPO_TAG_NAME}")
else()
  set(version "v0.7.2")
endif()

string(REGEX REPLACE "^v" "" version "${version}")

### Main project ###
project(drishtisdk VERSION ${version})

message("CMAKE_CXX_COMPILER: ${CMAKE_CXX_COMPILER}")
message("CMAKE_C_COMPILER: ${CMAKE_C_COMPILER}")

#####################################################################
## IMPORTANT: Set platform variables after project() and before
## the first hunter_add_package() call. The cmake/Hunter/config.cmake
## file will be parsed at the time of the first hunter_add_package()
## call, so we need to populate system variable before that.
## Do note move this!!!!!!!!
#####################################################################
string(COMPARE EQUAL "${CMAKE_SYSTEM_NAME}" "Linux" is_linux)

####################################################################
## Doxygen
####################################################################

## https://tty1.net/blog/2014/cmake-doxygen_en.html
find_package(Doxygen)
option(DRISHTI_BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(DRISHTI_BUILD_DOCUMENTATION)
  include(drishti_configure_doxygen)
  drishti_configure_doxygen()
endif()

### Add Sugar packages for cmake wrappers ###
hunter_add_package(Sugar)
include("${SUGAR_ROOT}/cmake/Sugar")
include(sugar_include)

include(drishti_set_cxx_flags)
drishti_set_cxx_flags()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

if(XCODE)
  set(CMAKE_CONFIGURATION_TYPES "Debug;Release;MinSizeRel;RelWithDebInfo")
endif()

message("** -- ** ${CMAKE_CXX_FLAGS}")
message("** -- ** ${CMAKE_CXX_FLAGS_RELEASE}")
message("** -- ** ${CMAKE_CXX_FLAGS_DEBUG}")
message("+++ ${CMAKE_CONFIGURATION_TYPES} +++" )
message("--- ${CMAKE_BUILD_TYPE} ---")

message("CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}")
message("CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER}")
message("OPENMP CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# The various compiler checks take > 10 seconds on ios "Try OpenMP CXX flag = [-openmp]" in xcode
# TODO: Investigate why this is so much slower with ios toolchain

# OpenMP also causes a compiler error in Eigen on vs-14-2015
# http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1131

if(NOT XCODE AND NOT MSVC) # We'll never have openmp on ios, and this test is really slow
  find_package( OpenMP )
  if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()

include(drishti_mirror_definition)

option(DRISHTI_BUILD_REGRESSION_SIMD "Build regression using SIMD." ON)
drishti_mirror_definition(DRISHTI_BUILD_REGRESSION_SIMD)

option(DRISHTI_BUILD_REGRESSION_FIXED_POINT "Build regression using fixed point." ON)
drishti_mirror_definition(DRISHTI_BUILD_REGRESSION_FIXED_POINT)

option(DRISHTI_BUILD_MIN_SIZE "Build minimum size lib (exclude training)" OFF)
drishti_mirror_definition(DRISHTI_BUILD_MIN_SIZE)

option(BUILD_SHARED_LIBS "Build package with shared libraries." OFF)

# Use static SDK build for initial asan tests.
option(DRISHTI_BUILD_SHARED_SDK "Build shared SDK" ON)
option(DRISHTI_BUILD_C_INTERFACE "Build C API" OFF)
option(DRISHTI_BUILD_EXAMPLES "Build the examples" OFF)
option(DRISHTI_BUILD_ASAN_TEST "Intentional ASAN test" OFF)
option(DRISHTI_BUILD_TESTS "Build unit tests" ON)
option(DRISHTI_BUILD_ACF "Drishti ACF lib." ON)
option(DRISHTI_BUILD_FACE "Drishti face lib." ON)
option(DRISHTI_BUILD_HCI "Drishti video and HCI lib." ON)

# 3rd party libraries
option(DRISHTI_BUILD_DEST "Build dest lib" OFF)
option(DRISHTI_BUILD_EOS "EOS 2D-3D fitting" OFF) # duplicate symbols

message("DRISHTI_BUILD_EOS (1) ${DRISHTI_BUILD_EOS}")

if(DRISHTI_BUILD_TESTS)
  if(ANDROID OR IOS)
    # FIXME: https://github.com/elucideye/drishti/issues/58
  else()
    enable_testing()
  endif()
  hunter_add_package(GTest)
  find_package(GTest CONFIG REQUIRED)
endif()

option(DRISHTI_BUILD_OGLES_GPGPU "Build with OGLES_GPGPU" OFF)
drishti_mirror_definition(DRISHTI_BUILD_OGLES_GPGPU)

# Note: OGLES_GPGPU handling moved to src folder
option(DRISHTI_BUILD_QT "Build with Qt" OFF)
drishti_mirror_definition(DRISHTI_BUILD_QT)

if(DRISHTI_BUILD_QT)
  message("=== BUILD QT #!#!@#$ ===== ")
  # Do this first for success/fail
  hunter_add_package(Qt COMPONENTS qtmultimedia qtdeclarative qtsensors qtquickcontrols)
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5Multimedia REQUIRED)
  find_package(Qt5MultimediaWidgets REQUIRED)
  find_package(Qt5OpenGL REQUIRED)
  find_package(Qt5Qml REQUIRED)
  find_package(Qt5Quick REQUIRED)
  find_package(Qt5QuickTest REQUIRED)
  find_package(Qt5QuickWidgets REQUIRED)
  find_package(Qt5Sensors REQUIRED) # for rotation
  #find_package(Qt5QuickControls REQUIRED) #  QtQuick.Control
endif()

if(IOS OR ANDROID)
  add_definitions(-DDRISHTI_OPENGL_ES=1)
else()
  add_definitions(-DDRISHTI_OPENGL_ES=0)
endif()

# XGBoost has replaced all previous regression
add_definitions(-DHAS_XGBOOST=1)

message("CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

#######################################################################################
#### https://github.com/forexample/github-binary-release/blob/master/CMakeLists.txt ###
#######################################################################################

include(InstallRequiredSystemLibraries)

include("cmake/pack.cmake") # includes Cpack

if(travis_deploy OR appveyor_deploy)
  string(COMPARE EQUAL "$ENV{CONFIG}" "Debug" debug_build)
  if(debug_build)
    set(CPACK_PACKAGE_NAME "${PROJECT_NAME}-$ENV{TOOLCHAIN}-Debug")
  else()
    set(CPACK_PACKAGE_NAME "${PROJECT_NAME}-$ENV{TOOLCHAIN}")
  endif()
endif()

#######################################################################################

# Include macros:
include(drishti_print_env_var)
include(ExternalProject)
include(drishti_always_full_rpath)
include(GenerateExportHeader)
include(drishti_option)

include(CheckTypeSize)
check_type_size("__fp16" FP16)
if(HAVE_FP16)
  add_definitions(-DHAVE_FP16=1)
  add_definitions(-DHAVE_HALF_FLOAT=1)
  add_definitions(-DDO_SQUEEZE=1)
endif()

#####################
### RPATH (BEGIN) ###
#####################
#s see: http://www.cmake.org/Wiki/CMake_RPATH_handling ("Always full RPATH")
drishti_always_full_rpath()

#####################
### Visibility    ###
#####################

# Always hide symbols within this project:
set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
set(CMAKE_CXX_VISIBILITY_PRESET hidden)

set(CMAKE_XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN "YES")
set(CMAKE_XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN "YES")

##############
### cotire ###
##############
# https://github.com/sakra/cotire
option(DRISHTI_COTIRE  "Use cotire single compilation unit build" OFF)
if(DRISHTI_COTIRE)
  include(cotire)
endif()

##################
### PACKAGES #####
##################

if(NOT MSVC)
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=900")
endif()

# Look for WITH_JPEG=ON in Hunter/config.cmake
# Dependency for OpenCV and cvmatio. Usually dependent packages are
# installed automatically but current hardcoded version of OpenCV is not
# hunterized enough so it doesn't know about Jpeg, i.e. there is no
# `hunter_add_package(Jpeg)`.  But Jpeg is present in fresh version of Hunter with
# custom FindJPEG.cmake module which expect JPEGConfig.cmake installed.
# OpenCV do use `find_package(JPEG)` without `hunter_add_package(Jpeg)`, hence
# FindJPEG.cmake exits with error "JPEG::jpeg not found". As a workaround
# we manually download Jpeg before OpenCV.
# TODO: remove after OpenCV version updated
if(is_linux)
  hunter_add_package(Jpeg)
  find_package(JPEG CONFIG REQUIRED)
endif()

### spdlog: Added logging library
hunter_add_package(spdlog)
find_package(spdlog CONFIG REQUIRED)

include(drishti_thread_local_storage)
drishti_thread_local_storage(HAVE_THREAD_LOCAL_STORAGE)

### Thread-pool-cpp
drishti_option(DRISHTI_USE_THREAD_POOL_CPP "Use thread-pool-cpp submodule" ON IF(HAVE_THREAD_LOCAL_STORAGE))
option(DRISHTI_USE_LOCAL_THREAD_POOL_CPP "Use thread-pool-cpp submodule" OFF)
  
if(DRISHTI_USE_LOCAL_THREAD_POOL_CPP)
  add_subdirectory(src/3rdparty/thread-pool-cpp)
  add_library(thread-pool-cpp::thread-pool-cpp ALIAS thread-pool-cpp)
else()
  hunter_add_package(thread-pool-cpp)
  find_package(thread-pool-cpp CONFIG REQUIRED)
endif()

### Cereal
hunter_add_package(cereal)
find_package(cereal CONFIG REQUIRED)

hunter_add_package(OpenCV)
find_package(OpenCV REQUIRED)

include_directories("${OpenCV_INCLUDE_DIRS}")
message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
message("OpenCV_LIB_COMPONENTS = ${OpenCV_LIB_COMPONENTS}")
message("OpenCV_LIBS = ${OpenCV_LIBS}")
message("${CMAKE_MODULE_PATH}")

### Boost ###
hunter_add_package(Boost COMPONENTS filesystem system serialization iostreams)
find_package(Boost CONFIG REQUIRED filesystem system serialization iostreams)
include_directories(${Boost_INCLUDE_DIRS})
message("Boost_INCLUDE_DIRS =  ${Boost_INCLUDE_DIRS}; Boost_LIBRARIES =  ${Boost_LIBRARIES}")
set(DRISHTI_SDK_BOOST_LIBS
  Boost::filesystem
  Boost::system
  Boost::serialization
  Boost::iostreams
  )

### DRISHTI_USE_TEXT_ARCHIVES
option(DRISHTI_USE_TEXT_ARCHIVES "Support TEXT archives for serialization (extra size)" OFF)
drishti_mirror_definition(DRISHTI_USE_TEXT_ARCHIVES)

### DLIB: dlib (Header only + no CMakeLists.txt)
hunter_add_package(dlib)
find_package(dlib CONFIG REQUIRED)
include_directories(${DLIB_INCLUDE_DIRS})
add_definitions(-DDLIB_NO_GUI_SUPPORT=1)

### Eigen3 (Header only)
hunter_add_package(Eigen)
find_package(Eigen REQUIRED) # <== not finding hunter provided eigen
message("EIGEN3_INCLUDE_DIR = ${EIGEN3_INCLUDE_DIR}")

### CVMATIO
if(DRISHTI_BUILD_ACF)
  hunter_add_package(cvmatio)
  find_package(cvmatio REQUIRED CONFIG)
endif()

#####################
### drishti_world ###
#####################

# This will build a static library using all specified modules
option(DRISHTI_BUILD_WORLD "Build single library" ON)

### Faster build itmes
if(XCODE)
  option(DRISHTI_DISABLE_DSYM "Disable generation of dSYM information" ON)
  if(DRISHTI_DISABLE_DSYM)
    set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf")
  endif()
  set(CMAKE_XCODE_ATTRIBUTE_ENABLE_BITCODE NO)

  # RelWithDebInfo should always have symbols
  set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT[variant=RelWithDebInfo] "dwarf-with-dsym")
  set(CMAKE_XCODE_ATTRIBUTE_GENERATE_PROFILING_CODE[variant=RelWithDebInfo] "YES")

  set(CMAKE_XCODE_ATTRIBUTE_CLANG_STATIC_ANALYZER_MODE_ON_ANALYZE_ACTION "Shallow")
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_STATIC_ANALYZER_MODE "Shallow")

endif()

if(NOT IOS AND NOT ANDROID)
  find_program(CMAKE_MEMORYCHECK_COMMAND valgrind)
  set(CMAKE_MEMORYCHECK_COMMAND_OPTIONS
    "--trace-children=yes"
    "--leak-check=full"
    "--tool=memcheck"
    "--show-leak-kinds=definite,possible"
    "--track-origins=yes"
    "--log-file=${CMAKE_SOURCE_DIR}/_logs/valgrind_report.txt"
    )
  find_file(HAVE_VALGRIND "valgrind")
  message("HAVE_VALGRIND: ${HAVE_VALGRIND}")

endif()

include_directories(src/lib)

# Legacy public interface
include_directories(src/lib/drishti)

### Setup install names for later use:
set(CMAKE_PROJECT_NAME drishti) # overwrite drishtisdk
set(drishti_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(drishti_include_install_dir "include")
set(drishti_config_install_dir "share/${CMAKE_PROJECT_NAME}")
set(drishti_version_config "${drishti_generated_dir}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake")
set(drishti_project_config "${drishti_generated_dir}/${CMAKE_PROJECT_NAME}Config.cmake")
set(drishti_targets_export_name "${CMAKE_PROJECT_NAME}Targets")
set(drishti_namespace "${CMAKE_PROJECT_NAME}::")

### Download 'faces'
include(drishti_download_private_data)
drishti_download_private_data(
    URL "https://github.com/elucideye/drishti-faces/archive/v1.0.zip"
    SHA1 "dc6d1447e16501a684eba8ccd84750b9dbda7ac0"
    NAME "faces"
    DIR faces_dir
    PASSWORDS "${CMAKE_CURRENT_LIST_DIR}/passwords.cmake"
)
message("Faces directory: '${faces_dir}'")

### Download 'assets'
drishti_download_private_data(
    URL "https://github.com/elucideye/drishti-assets/archive/v1.0.tar.gz"
    SHA1 "d2246ca6af8f3355a7a96644ed9416cd188c4d8e"
    NAME "assets"
    DIR assets_dir
    PASSWORDS "${CMAKE_CURRENT_LIST_DIR}/passwords.cmake"
)
message("Assets directory: '${assets_dir}'")

add_subdirectory(src)
