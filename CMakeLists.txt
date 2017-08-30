# Copyright (c) 2014-2017, David Hirvonen
# All rights reserved.

cmake_minimum_required(VERSION 3.3)

#########################
### CMAKE_MODULE_PATH ###
#########################

set(drishti_upload_modules "${CMAKE_CURRENT_LIST_DIR}/drishti-upload/cmake/Modules")
list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")
list(APPEND CMAKE_MODULE_PATH "${drishti_upload_modules}")

############################
### HunterGate and cache ###
############################

# drishti specific hunter cache pacakges:
list(APPEND HUNTER_CACHE_SERVERS "https://github.com/elucideye/hunter-cache")

if(EXISTS "${drishti_upload_modules}")
  include(drishti_set_hunter_gate)
  drishti_set_hunter_version(DRISHTI_HUNTER_GATE_URL DRISHTI_HUNTER_GATE_SHA1)
else()
  # Release archive will not contain submodule 'drishti-upload'.
  # It's a valid case when archive used in Hunter.
  # URL/SHA1 arguments of HunterGate call will not be used while building
  # in this situation.
  set(DRISHTI_HUNTER_GATE_URL)
  set(DRISHTI_HUNTER_GATE_SHA1)
endif()

include("cmake/HunterGate.cmake")
HunterGate(
  URL "${DRISHTI_HUNTER_GATE_URL}"
  SHA1 "${DRISHTI_HUNTER_GATE_SHA1}"
  FILEPATH "${CMAKE_CURRENT_LIST_DIR}/drishti-upload/config.cmake"
  )

##########################
### CI Travis/Appveyor ###
##########################

string(COMPARE NOTEQUAL "$ENV{TRAVIS_TAG}" "" travis_deploy)
string(COMPARE EQUAL "$ENV{APPVEYOR_REPO_TAG}" "true" appveyor_deploy)

if(travis_deploy)
  set(version "$ENV{TRAVIS_TAG}")
elseif(appveyor_deploy)
  set(version "$ENV{APPVEYOR_REPO_TAG_NAME}")
else()
  set(version "v0.8.3") # note: rc
endif()

string(REGEX REPLACE "^v" "" version "${version}")

#######################
### Drishti project ###
#######################

project(drishtisdk VERSION ${version})

if(CMAKE_INTERPROCEDURAL_OPTIMIZATION)
  set(CMAKE_POLICY_DEFAULT_CMP0069 NEW) # for 3rd parties added by add_subdirectory
  cmake_policy(SET CMP0069 NEW)
endif()

set(DRISHTI_3RD_PARTY_DIR "${CMAKE_CURRENT_LIST_DIR}/src/3rdparty")
set(DRISHTI_APP_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/src/app")
set(DRISHTI_INCLUDE_DIRECTORIES "${CMAKE_CURRENT_LIST_DIR}/src/lib")

#####################################################################
## IMPORTANT: Set platform variables after project() and before
## the first hunter_add_package() call. The cmake/Hunter/config.cmake
## file will be parsed at the time of the first hunter_add_package()
## call, so we need to populate system variable before that.
## Do note move this!!!!!!!!
#####################################################################

# Warning: The following options relate to
# drishti-upload/config.cmake:
#
# DRISHTI_BUILD_MIN_SIZE:
#   This is included via drishti-upload/config.cmake
#
# DRISHTI_BUILD_MERGED_SDK: (see: DRISHTI_BUILD_OPENCV_WORLD)
#   Force DRISHTI_BUILD_OPENCV_WORLD=OFF since the merged
#   library introduces duplicate object names (filenames)
#   which causes errors when splitting libopencv_world.a
#   into object files.
#
# DRISHTI_BUILD_OPENCV_WORLD: (see: DRISHTI_BUILD_MERGED_SDK)
#   This is an override for the same setting in
#   drishti-upload/config.cmake.
#
# DRISHTI_BIULD_MIN_SIZE:
#   This can alter the hunter-package options for
#   various dependencies in order to produce smaller builds,
#   such as eliminating code related to model training.
#
# In typical CI builds we use the following settings:
#   HUNTER_USE_CACHE_SERVERS=ONLY
#   HUNTER_DISABLE_BUILDS=YES
#
# Note that any time we attempt to override a setting
# in drishti-upload/config.cmake, the config-id will
# often be different, which can invalidate the cache
# and the CI builds are likely to terminate and report
# an error.

option(DRISHTI_BUILD_SHARED_SDK "Build shared SDK" OFF)
option(DRISHTI_BUILD_MERGED_SDK "Merge all static dependencies" OFF)

option(DRISHTI_COPY_3RDPARTY_LICENSES "Install 3rd party licenses" OFF)

# See: drishti-upload/config.cmake
# option(DRISHTI_BUILD_MIN_SIZE "Build minimum size lib (exclude training)" ON)

if(DRISHTI_BUILD_MERGED_SDK)
  # Override the drishti-upload DRISHTI_BUILD_OPENCV_WORLD option, since the opencv_world
  # library contains duplicate object filenames that prevents a successful split + merge
  # due to limitations in the ar utility.
  option(DRISHTI_BUILD_OPENCV_WORLD "Build OpenCV world (monolithic lib)" OFF)
endif()

message("CMAKE_INSTALL_PREFIX: ${CMAKE_INSTALL_PREFIX}")

#######################
#### CMake includes ###
#######################

include(GenerateExportHeader)
include(drishti_bool_to_int)
include(drishti_option)
include(drishti_print_env_var)
include(drishti_set_cxx_flags)

drishti_set_cxx_flags()

string(COMPARE EQUAL "${CMAKE_SYSTEM_NAME}" "Linux" is_linux)

if(IOS OR ANDROID)
  set(DRISHTI_IS_MOBILE TRUE)
else()
  set(DRISHTI_IS_MOBILE FALSE)
endif()

set_property(GLOBAL PROPERTY USE_FOLDERS ON)

##############
## Doxygen ###
##############

## https://tty1.net/blog/2014/cmake-doxygen_en.html
find_package(Doxygen)
option(DRISHTI_BUILD_DOCUMENTATION "Create and install the HTML based API documentation (requires Doxygen)" ${DOXYGEN_FOUND})

if(DRISHTI_BUILD_DOCUMENTATION)
  include(drishti_configure_doxygen)
  drishti_configure_doxygen()
endif()

######################
### RPATH defaults ###
######################

# see: http://www.cmake.org/Wiki/CMake_RPATH_handling
include(drishti_set_rpath)
drishti_set_rpath()

###########################
### XCODE customization ###
###########################

if(XCODE)
  set(CMAKE_XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN "YES")
  set(CMAKE_XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN "YES")

  set(CMAKE_XCODE_ATTRIBUTE_ENABLE_BITCODE NO)
  option(DRISHTI_DISABLE_DSYM "Disable generation of dSYM information" ON)
  if(DRISHTI_DISABLE_DSYM)
    message("CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT dwarf")
    set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf")
    set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT[variant=Release] "dwarf")
  else()
    message("CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT dwarf-with-dsym")
    set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT "dwarf-with-dsym")
    set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT[variant=Release] "dwarf-with-dsym")

    # Generate debug symbols but strip them (i.e., clang or gcc)
    set(CMAKE_XCODE_ATTRIBUTE_GCC_GENERATE_DEBUGGING_SYMBOLS "YES")
    set(CMAKE_XCODE_ATTRIBUTE_GCC_GENERATE_DEBUGGING_SYMBOLS[variant=Release] "YES")

    # Add -g to force dSYM generation
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -g")
  endif()

  # RelWithDebInfo should always have symbols
  set(CMAKE_XCODE_ATTRIBUTE_DEBUG_INFORMATION_FORMAT[variant=RelWithDebInfo] "dwarf-with-dsym")
  set(CMAKE_XCODE_ATTRIBUTE_GENERATE_PROFILING_CODE[variant=RelWithDebInfo] "YES")

  # Minimize static analyzer time
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_STATIC_ANALYZER_MODE_ON_ANALYZE_ACTION "Shallow")
  set(CMAKE_XCODE_ATTRIBUTE_CLANG_STATIC_ANALYZER_MODE "Shallow")

endif()

# OpenMP compiler checks take > 10 seconds on ios "Try OpenMP CXX flag = [-openmp]" in xcode
# TODO: Investigate why this is so much slower with ios toolchain
# OpenMP also causes a compiler error in Eigen on vs-14-2015
# http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1131
if(NOT XCODE AND NOT MSVC) # We'll never have openmp on ios, and this test is really slow
  # This is only used for xgboost training and it will be removed once the
  # xgboost interface is updated.
  find_package( OpenMP )
  if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()

#######################
### Packaging/CPack ###
#######################
# see https://github.com/forexample/github-binary-release/blob/master/CMakeLists.txt

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

###############################
### Serialization/Archives ####
###############################

# See corresponding target_compile_definitions() in src/libdrishti/CMakeLists.txt
# Serialization options pertain to both library builds and integration testing.
# In the case of integration testing, the user must specify serialization options
# that are compatible with the compiled libraries.
option(DRISHTI_SERIALIZE_WITH_CVMATIO "Perform serialization with cvmatio" ON)

################
#### Testing ###
################

# DRISHTI_BUILD_TESTS:
# ${DRISHTISDK}/src/lib/drishti/<module>/ut/
# ${DRISHTISDK}/src/tests

# DRISHIT_BUILD_INTEGRATION_TESTS:
# ${DRISHTISDK}/src/examples/integration/

option(DRISHTI_BUILD_BENCHMARKS "Build internal benchmarks (size and speed)." OFF)
option(DRISHTI_BUILD_TESTS "Build and run internal unit tests." OFF)
option(DRISHTI_BUILD_INTEGRATION_TESTS "Build and run tests for installed libraries." OFF)

string(COMPARE EQUAL "$ENV{TRAVIS}" "true" travis_ci)
string(COMPARE EQUAL "$ENV{APPVEYOR}" "True" appveyor_ci)
if(travis_ci OR appveyor_ci)
  set(DRISHTI_CI TRUE)
else()
  set(DRISHTI_CI FALSE)
endif()

if(ANDROID OR NOT ${DRISHTI_CI})
  set(DRISHTI_DO_GPU_TESTING TRUE)
else()
  set(DRISHTI_DO_GPU_TESTING FALSE)
endif()

if(DRISHTI_BUILD_TESTS OR DRISHTI_BUILD_INTEGRATION_TESTS)
  if(IOS AND DRISHTI_CI)
    # do not run test on CI (TODO: remote device testing)
  else()
    enable_testing()
  endif()
  hunter_add_package(gauze)
  find_package(gauze CONFIG REQUIRED)

  hunter_add_package(GTest)
  find_package(GTest CONFIG REQUIRED)
endif()

#################
### Test data ###
#################

hunter_add_package(drishti_assets)
find_package(drishti_assets CONFIG REQUIRED)

set(DRISHTI_ASSETS_VARS
  DRISHTI_ASSETS_FACE_DETECTOR
  DRISHTI_ASSETS_FACE_DETECTOR_MEAN
  DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR
  DRISHTI_ASSETS_EYE_MODEL_REGRESSOR
  )

foreach(model ${DRISHTI_ASSETS_VARS})
  # Override assets using corresponding environment variables if present:
  if(DEFINED ENV{${model}})
    message("RESET ${model} = $ENV{${model}}")
    unset(${model} CACHE)
    set(${model} $ENV{${model}})
  endif()
endforeach()

hunter_add_package(drishti_faces)
find_package(drishti_faces CONFIG REQUIRED)

#######################
### Compiler checks ###
#######################

### If true C++11 thread_local support exists, we will use it:
include(drishti_thread_local_storage_cpp11)
drishti_thread_local_storage_cpp11(DRISHTI_HAVE_THREAD_LOCAL_STORAGE)
if(NOT DRISHTI_HAVE_THREAD_LOCAL_STORAGE)
  # Else, we will check for backups
  include(drishti_thread_local_storage)
  drishti_thread_local_storage(DRISHTI_HAVE_THREAD_LOCAL_STORAGE)
endif()
message("DRISHTI_HAVE_THREAD_LOCAL_STORAGE : ${DRISHTI_HAVE_THREAD_LOCAL_STORAGE}")

###########################
### Print compiler info ###
###########################

message("CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}")
message("CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER}")
message("** -- ** ${CMAKE_CXX_FLAGS}")
message("** -- ** ${CMAKE_CXX_FLAGS_RELEASE}")
message("** -- ** ${CMAKE_CXX_FLAGS_DEBUG}")
message("+++ ${CMAKE_CONFIGURATION_TYPES} +++" )
message("--- ${CMAKE_BUILD_TYPE} ---")

####################################
### Sugar: collect cmake sources ###
####################################

hunter_add_package(Sugar)
include("${SUGAR_ROOT}/cmake/Sugar")
include(sugar_include)

################################################
### Drishti integration (post-install) tests ###
################################################

if(DRISHTI_BUILD_INTEGRATION_TESTS)
  # Re-use drishti test interface for integration testing
  add_subdirectory(src/lib/drishti/testlib)
  add_subdirectory(src/examples/integration)
  return()
endif()

### ^
### | Settings above this line apply to both drishti library
### | and drishti integration testing of installed libraries.
### +---------------------------------------------------------
### | Settings below this line apply only to drishti libraries.
### |
### V

##############
### cotire ###
##############

# https://github.com/sakra/cotire
option(DRISHTI_COTIRE "Use cotire single compilation unit build" OFF)
if(DRISHTI_COTIRE)
  include(cotire)
endif()

option(BUILD_SHARED_LIBS "Build package with shared libraries." OFF)

#################################
### Drishti LIB configuration ###
#################################

# This will build a static library using all specified modules
option(DRISHTI_INSTALL "Run package-config installation" ON)
option(DRISHTI_BUILD_WORLD "Build single drishti library/module (internal)" ON)
option(DRISHTI_BUILD_C_INTERFACE "Build C API" OFF)
option(DRISHTI_BUILD_EXAMPLES "Build the examples" ON)
option(DRISHTI_BUILD_ASAN_TEST "Intentional ASAN test" OFF)
option(DRISHTI_BUILD_ACF "Drishti ACF lib" ON)
option(DRISHTI_BUILD_FACE "Drishti face lib" ON)
option(DRISHTI_BUILD_HCI "Drishti video and HCI lib" ON)
option(DRISHTI_BUILD_REGRESSION_SIMD "Build multivariate gradient boosting using SIMD" ON)
option(DRISHTI_BUILD_REGRESSION_FIXED_POINT "Build multivariate gradient boosting using fixed point" ON)

# 3rd party libraries
option(DRISHTI_BUILD_DEST "Build dest lib" OFF)
option(DRISHTI_BUILD_EOS "EOS 2D-3D fitting" OFF) # duplicate symbols
option(DRISHTI_BUILD_CV_ML "Build OpenCV ML classes" OFF)

##################################################
### Installation/packaging paths and variables ###
##################################################
set(CMAKE_PROJECT_NAME drishti) # overwrite drishtisdk
set(drishti_generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(drishti_include_install_dir "include")
set(drishti_config_install_dir "lib/cmake/${CMAKE_PROJECT_NAME}")
set(drishti_version_config "${drishti_generated_dir}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake")
set(drishti_project_config "${drishti_generated_dir}/${CMAKE_PROJECT_NAME}Config.cmake")
set(drishti_targets_export_name "${CMAKE_PROJECT_NAME}Targets")
set(drishti_namespace "${CMAKE_PROJECT_NAME}::")

##################################
### Includes (to be deprecated ###
##################################

add_subdirectory(src)

#################################
### Print the full asset list ###
#################################
foreach(model ${DRISHTI_ASSETS_VARS})
  message(">>> ${model} = ${${model}}")
endforeach()
