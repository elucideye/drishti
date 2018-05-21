################################################################################
# This directory tree contains examples built on the dependency free public SDK
# contained in src/lib/drishti/drishti.  This layer contains various simple
# primitive types to support easy integration as a shared library
# (dynamic framework) without requiring OpenCV types or types from other
# dependencies which may change.  No dynamic memory allocations are exported
# across this API line (including STL to avoid libc++ vs libstdc++ conflicts,
# etc).
################################################################################

if(NOT DRISHTI_BUNDLE_SDK_SAMPLES)

  set(DRISHTI_ROOT_DIR "${CMAKE_CURRENT_LIST_DIR}/../../..")

  cmake_minimum_required(VERSION 3.9) # CMAKE_INTERPROCEDURAL_OPTIMIZATION

  # https://docs.hunter.sh/en/latest/reference/user-variables.html#hunter-keep-package-sources
  option(HUNTER_KEEP_PACKAGE_SOURCES "Keep installed package sources for debugging (caveat...)" OFF)

  list(APPEND CMAKE_MODULE_PATH "${DRISHTI_ROOT_DIR}/cmake/Modules")

  # Test SDK samples using the Hunter packages (release based)
  include("${DRISHTI_ROOT_DIR}/cmake/default_options.cmake")
  include("${DRISHTI_ROOT_DIR}/cmake/HunterGate.cmake")
  
  HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.20.79.tar.gz"
    SHA1 "428c555b2205ce6aa94707a97c18be7afebecee2"
    FILEPATH "${DRISHTI_ROOT_DIR}/cmake/Hunter/config.cmake"
  )

  project(drishti_hunter_test VERSION 0.0.1)

  # add the drishti package
  hunter_add_package(drishti)
  find_package(drishti CONFIG REQUIRED)

endif()

if(NOT DEFINED DRISHTI_HAVE_TO_STRING)
  try_compile(DRISHTI_HAVE_TO_STRING "${CMAKE_BINARY_DIR}/compile_tests" "${DRISHTI_ROOT_DIR}/cmake/to_string.cpp")
endif()
if(NOT DEFINED DRISHTI_HAVE_LOCALECONV)
  try_compile(DRISHTI_HAVE_LOCALECONV "${CMAKE_BINARY_DIR}/compile_tests" "${DRISHTI_ROOT_DIR}/cmake/localeconv.cpp")
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
