################################################################################
# This directory tree contains examples built on the dependency free public SDK
# contained in src/lib/drishti/drishti.  This layer contains various simple
# primitive types to support easy integration as a shared library
# (dynamic framework) without requiring OpenCV types or types from other
# dependencies which may change.  No dynamic memory allocations are exported
# across this API line (including STL to avoid libc++ vs libstdc++ conflicts,
# etc).
################################################################################

cmake_minimum_required(VERSION 3.9) # CMAKE_INTERPROCEDURAL_OPTIMIZATION

include("cmake/include/init.cmake")

project(drishti_hunter_test VERSION 0.0.1)

#################
### Languages ###
#################

if(IOS)
  # This must be in the top/highest level directory (really)
  set(CMAKE_Swift_LANGUAGE_VERSION 4.1)
  set(CMAKE_XCODE_ATTRIBUTE_SWIFT_VERSION "4.1")
  enable_language(Swift)
endif()

# * If build as a child project then parent will set this to 'OFF' (see
#   config.cmake which will be loaded on first 'hunter_add_package')
# * If build as a stand alone project then option will be 'ON' and 'GIT_SELF'
#   will be used for Drishti
option(DRISHTI_DRISHTI_AS_SELF "Use drishti as GIT_SELF" ON)

if(NOT DEFINED DRISHTI_HAVE_TO_STRING)
  try_compile(DRISHTI_HAVE_TO_STRING
    "${CMAKE_BINARY_DIR}/try_compile"
    "${CMAKE_CURRENT_LIST_DIR}/cmake/to_string.cpp"
  )
endif()

if(NOT DEFINED DRISHTI_HAVE_LOCALECONV)
  try_compile(DRISHTI_HAVE_LOCALECONV
    "${CMAKE_BINARY_DIR}/try_compile"
    "${CMAKE_CURRENT_LIST_DIR}/cmake/localeconv.cpp"
  )
endif()

################################
### Application dependencies ###
################################

hunter_add_package(OpenCV) # for image IO
find_package(OpenCV REQUIRED)

hunter_add_package(cxxopts) # for CLI
find_package(cxxopts CONFIG REQUIRED)

hunter_add_package(spdlog) # for logging
find_package(spdlog CONFIG REQUIRED)

if(NOT TARGET drishti::drishti)
  # add the drishti package, possibly using GIT_SELF feature
  hunter_add_package(drishti)
  find_package(drishti CONFIG REQUIRED)
endif()

set(base_deps drishti::drishti cxxopts::cxxopts ${OpenCV_LIBS} spdlog::spdlog)

if(DRISHTI_TEST_BUILD_TESTS)
  hunter_add_package(gauze) # for tests
  find_package(gauze CONFIG REQUIRED)
  list(APPEND base_deps gauze::gauze)
endif()

###############
### Samples ###
###############

add_subdirectory(eye)
add_subdirectory(face)
add_subdirectory(facefilter)
