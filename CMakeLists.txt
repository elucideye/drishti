# Copyright (c) 2014-2017, David Hirvonen
# All rights reserved.

cmake_minimum_required(VERSION 3.9) # CMAKE_INTERPROCEDURAL_OPTIMIZATION

include("src/examples/cmake/include/init.cmake")

#########################
### CMAKE_MODULE_PATH ###
#########################

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")

#######################
### Drishti project ###
#######################

project(drishtisdk VERSION 0.12.4)

#################
### Languages ###
#################

if(IOS)
  # This must be in the top/highest level directory (really)
  set(CMAKE_Swift_LANGUAGE_VERSION 4.1)
  set(CMAKE_XCODE_ATTRIBUTE_SWIFT_VERSION "4.1")
  enable_language(Swift)
endif()  

set(CMAKE_POLICY_DEFAULT_CMP0069 NEW) # for 3rd parties added by add_subdirectory

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

option(DRISHTI_BUILD_SHARED_SDK "Build shared SDK" OFF)
option(DRISHTI_BUILD_MERGED_SDK "Merge all static dependencies" OFF)

option(DRISHTI_COPY_3RDPARTY_LICENSES "Install 3rd party licenses" ON)

if(DRISHTI_BUILD_MERGED_SDK)
  # Override the drishti-upload DRISHTI_BUILD_OPENCV_WORLD option, since the opencv_world
  # library contains duplicate object filenames that prevents a successful split + merge
  # due to limitations in the ar utility.
  option(DRISHTI_BUILD_OPENCV_WORLD "Build OpenCV world (monolithic lib)" OFF)
endif()

option(DRISHTI_BUILD_ACF "Drishti ACF lib" ON)

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

####################
## Version check ###
####################

hunter_add_package(check_ci_tag)
find_package(check_ci_tag CONFIG REQUIRED)
check_ci_tag()

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

#######################
### Packaging/CPack ###
#######################
# see https://github.com/forexample/github-binary-release/blob/master/CMakeLists.txt

include(InstallRequiredSystemLibraries)
include("${CMAKE_CURRENT_LIST_DIR}/cmake/pack.cmake") # includes Cpack

string(COMPARE NOTEQUAL "$ENV{TRAVIS_TAG}" "" travis_deploy)
string(COMPARE EQUAL "$ENV{APPVEYOR_REPO_TAG}" "true" appveyor_deploy)

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
option(DRISHTI_SERIALIZE_WITH_CVMATIO "Perform serialization with cvmatio" OFF)

################
#### Testing ###
################

# DRISHTI_BUILD_TESTS:
# ${DRISHTISDK}/src/lib/drishti/<module>/ut/
# ${DRISHTISDK}/src/tests

option(DRISHTI_BUILD_BENCHMARKS "Build internal benchmarks (size and speed)." OFF)
option(DRISHTI_BUILD_TESTS "Build and run internal unit tests." OFF)

option(DRISHTI_HAS_GPU "Drishti has GPU" ON)

if(DRISHTI_HAS_GPU AND DRISHTI_BUILD_TESTS)
  set(DRISHTI_DO_GPU_TESTING TRUE)
else()
  set(DRISHTI_DO_GPU_TESTING FALSE)
endif()

if(DRISHTI_BUILD_TESTS)
  enable_testing()
  hunter_add_package(gauze)
  find_package(gauze CONFIG REQUIRED)

  if(ANDROID AND NOT CMAKE_HOST_WIN32)
    hunter_add_package(Android-SDK)
    set(old "${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")
    set(new "${CMAKE_CURRENT_BINARY_DIR}/adb")
    if(NOT EXISTS "${new}")
      execute_process(
          COMMAND ${CMAKE_COMMAND} -E create_symlink ${old} ${new}
          RESULT_VARIABLE result
      )
      if(NOT result EQUAL "0")
        message(FATAL_ERROR "Can't create symlink")
      endif()
    endif()
  endif()

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

hunter_add_package(sugar)
find_package(sugar CONFIG REQUIRED)

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
option(DRISHTI_BUILD_C_INTERFACE "Build C API" OFF)
option(DRISHTI_BUILD_UTILITIES "Build the utilities (may use private api)" ON)
option(DRISHTI_BUILD_EXAMPLES "Build the examples (public/relocatable api)" ON)
option(DRISHTI_BUILD_ASAN_TEST "Intentional ASAN test" OFF)
option(DRISHTI_BUILD_FACE "Drishti face lib" ON)
option(DRISHTI_BUILD_HCI "Drishti video and HCI lib" ON)
drishti_option(DRISHTI_BUILD_REGRESSION_SIMD "Build multivariate gradient boosting using SIMD" ON IF(${DRISHTI_MOBILE}))
drishti_option(DRISHTI_BUILD_REGRESSION_FIXED_POINT "Build multivariate gradient boosting using fixed point" ON IF(${DRISHTI_MOBILE}))

# 3rd party libraries
option(DRISHTI_BUILD_DEST "Build dest lib" OFF)
option(DRISHTI_BUILD_EOS "EOS 2D-3D fitting" OFF) # duplicate symbols

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

#### std::to_string ####
try_compile(
    DRISHTI_HAVE_TO_STRING
    "${CMAKE_BINARY_DIR}/try_compile"
    "${CMAKE_CURRENT_LIST_DIR}/src/examples/cmake/to_string.cpp"
)

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
