cmake_minimum_required(VERSION 3.0)

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_LIST_DIR}/cmake/Modules")

include("cmake/HunterGate.cmake")
### Hunter snapshot that will be used ###
HunterGate(
    URL "https://github.com/ruslo/hunter/archive/v0.15.18.tar.gz"
    SHA1 "af15f396b3b7e90e7956a377c633106476567970"
    LOCAL # load `${CMAKE_CURRENT_LIST_DIR}/cmake/Hunter/config.cmake`
    )

string(COMPARE NOTEQUAL "$ENV{TRAVIS_TAG}" "" travis_deploy)
string(COMPARE EQUAL "$ENV{APPVEYOR_REPO_TAG}" "true" appveyor_deploy)

if(travis_deploy)
  set(version "$ENV{TRAVIS_TAG}")
elseif(appveyor_deploy)
  set(version "$ENV{APPVEYOR_REPO_TAG_NAME}")
else()
  set(version "v0.7.1")
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
include(drishti_check_platforms)
drishti_check_platforms()

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

# set(CMAKE_C_COMPILER "/usr/local/bin/clang-omp")
# set(CMAKE_CXX_COMPILER "/usr/local/bin/clang-omp++")
# set(CMAKE_XCODE_ATTRIBUTE_CC "/usr/local/bin/clang-omp")

message("CMAKE_CXX_COMPILER ${CMAKE_CXX_COMPILER}")
message("CMAKE_C_COMPILER ${CMAKE_CXX_COMPILER}")
message("OPENMP CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")

# The various compiler checks take > 10 seconds on ios "Try OpenMP CXX flag = [-openmp]" in xcode
# TODO: Investigate why this is so much slower with ios toolchain
if(NOT XCODE) # We'll never have openmp on ios, and this test is really slow
  find_package( OpenMP )
  if(OPENMP_FOUND)
    message("OPENMP FOUND")
    set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
    set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${OpenMP_EXE_LINKER_FLAGS}")
  endif()
endif()

option(DRISHTI_BUILD_REGRESSION_SIMD "Build regression using SIMD." ON)
if(DRISHTI_BUILD_REGRESSION_SIMD)
  add_definitions(-DBUILD_REGRESSION_SIMD=1)
else()
  add_definitions(-DBUILD_REGRESSION_SIMD=0)
endif()

option(DRISHTI_BUILD_REGRESSION_FIXED_POINT "Build regression using fixed point." ON)
if(DRISHTI_BUILD_REGRESSION_FIXED_POINT)
  add_definitions(-DBUILD_REGRESSION_FIXED_POINT=1)
else()
  add_definitions(-DBUILD_REGRESSION_FIXED_POINT=0)
endif()

option(DRISHTI_BUILD_MIN_SIZE "Build minimum size lib (exclude training)" OFF)
if(DRISHTI_BUILD_MIN_SIZE)
  add_definitions(-DDRISHTI_BUILD_MIN_SIZE=1)
else()
  add_definitions(-DDRISHTI_BUILD_MIN_SIZE=0)
endif()

option(BUILD_SHARED_LIBS "Build package with shared libraries." OFF)
option(DRISHTI_BUILD_EXAMPLES "Build the examples" OFF)
option(BUILD_PUBLIC_SDK "Build the public drishti SDK" ON)
option(BUILD_DRISHTI_ACF "Drishti ACF lib." OFF)
option(BUILD_DRISHTI_FACE "Drishti face lib." OFF)

# 3rd party libraries
option(BUILD_THEIA_SOLVERS "Build theia solvers lib" OFF)
option(BUILD_DEST "Build dest lib" OFF)
option(BUILD_TESTS "Build unit tests" ON)
option(BUILD_OGLES_GPGPU "Build with OGLES_GPGPU" OFF)

# Note: OGLES_GPGPU handling moved to src folder
option(BUILD_QT "Build with Qt" OFF)

if(BUILD_QT)
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
  set(DRISHTI_OPENGL_ES TRUE)
  add_definitions(-DDRISHTI_OPENGL_ES=1)
else()
  set(DRISHTI_OPENGL_ES FALSE)
  add_definitions(-DDRISHTI_OPENGL_ES=0)
endif()

# XGBoost has replaced all previous regression
add_definitions(-DHAS_XGBOOST=1)

# if(BUILD_DRISHTI_MOTION)
#   add_definitions(-DHAS_DRISHTI_MOTION=1)
# endif()

# Desired behavior:
set(PREFIX "${CMAKE_CURRENT_BINARY_DIR}")
set(INSTALL_DIR "${CMAKE_CURRENT_BINARY_DIR}")
set(CMAKE_ARGS "-DCMAKE_INSTALL_PREFIX=${INSTALL_DIR}")

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
include(CTest)
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
### RPATH (END)   ###
#####################

#####################
### Visibility    ###
#####################

# Not working for clang
#set(CMAKE_VISIBILITY_INLINES_HIDDEN ON)
#set(CMAKE_CXX_VISIBILITY_PRESET hidden)

# Note: The brute force approach seems to be required to get xcode working:
#set(XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN "YES")
#set(XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN "YES")

#set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")
#set(CMAKE_C_FLAGS "${CMAKE_C_FLAGS} -fvisibility=hidden -fvisibility-inlines-hidden")

##################
### PACKAGES #####
##################

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -ftemplate-depth=900")

if(MSVC OR is_linux) # Look for WITH_JPEG=ON in Hunter/config.cmake
  # Dependency for OpenCV. Usually dependent packages installed automatically
  # but current hardcoded version of OpenCV is not hunterized enough so
  # don't know about Jpeg, i.e. there is no `hunter_add_package(Jpeg)`.
  # But Jpeg is present in fresh version of Hunter with
  # custom FindJPEG.cmake module which expect JPEGConfig.cmake installed.
  # OpenCV do use `find_package(JPEG)` without `hunter_add_package(Jpeg)`, hence
  # FindJPEG.cmake exits with error "JPEG::jpeg not found". As a workaround
  # we manually download Jpeg before OpenCV.
  # TODO: remove after OpenCV version updated
  hunter_add_package(Jpeg)
  find_package(JPEG CONFIG REQUIRED)
endif()

### spdlog: Added logging library
hunter_add_package(spdlog)
find_package(spdlog CONFIG REQUIRED)

include(CheckCSourceCompiles)

### Thread-pool-cpp
check_c_source_compiles("
#if defined (__GNUC__)
    #define ATTRIBUTE_TLS __thread
#elif defined (_MSC_VER)
    #define ATTRIBUTE_TLS __declspec(thread)
#else // !__GNUC__ && !_MSC_VER
    #error \"Define a thread local storage qualifier for your compiler/platform!\"
#endif
ATTRIBUTE_TLS int tls;
int main(void) {
    return 0;
}" HAVE_THREAD_LOCAL_STORAGE)

drishti_option(USE_THREAD_POOL_CPP "Use thread-pool-cpp submodule" ON IF(HAVE_THREAD_LOCAL_STORAGE))

option(USE_LOCAL_THREAD_POOL_CPP "Use thread-pool-cpp submodule" OFF)
if(USE_LOCAL_THREAD_POOL_CPP)
  add_definitions(-DUSE_LOCAL_THREAD_POOL=1)
  set(THREAD_POOL_LIB ) # header only (null)
  include_directories(src/3rdparty/thread-pool-cpp/thread_pool)
else()
  add_definitions(-DUSE_LOCAL_THREAD_POOL=0)
  hunter_add_package(thread-pool-cpp)
  find_package(thread-pool-cpp CONFIG REQUIRED)
  set(THREAD_POOL_LIB thread-pool-cpp::thread-pool-cpp)
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
if(BUILD_DRISHTI_ACF)
  hunter_add_package(cvmatio)
  find_package(cvmatio REQUIRED CONFIG)
endif()

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

 # get_cmake_property(_variableNames VARIABLES)
 # foreach (_variableName ${_variableNames})
 #     message(STATUS "VARIABLE: ${_variableName}=${${_variableName}}")
 # endforeach()
 # return()

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

include_directories(src/lib/drishti)
include_directories(src/lib/core)
include_directories(src/lib/solvers)
include_directories(src/lib/ml)
include_directories(src/lib/acf)
include_directories(src/lib/rcpr)
include_directories(src/lib/geometry)
include_directories(src/lib/graphics)
include_directories(src/lib/sensor)
include_directories(src/lib/face)
include_directories(src/lib/eye)

add_subdirectory(src)

message("${PROJECT_NAME}:")
message("-- system --")
message("  APPLE: ${APPLE}")
message("  iOS: ${IOS}")
message("-- configuration --")
message("  Build shared libraries: ${BUILD_SHARED_LIBS}")
message("-- packages --")
message("  Sugar: ${SUGAR_ROOT}/cmake/Sugar")
message("  ZLIB: ${ZLIB_DIR}")
message("  OpenCV: ${OpenCV_DIR}")
message("  CVMATIO: ${CVMATIO_DIR}")
message("  GPUImage: ${GPUImage_DIR}")
