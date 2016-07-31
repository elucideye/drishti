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

option(USE_LOCAL_XGBOOST "Use local xgboost" ON)
if(USE_LOCAL_XGBOOST)
  # Note: Use either XGBOOST
  set(SKIP_INSTALL ON CACHE BOOL "Avoid installation of submodule libraries")
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost")
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost/src")
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost)
  set(XGBOOST_LIB xgboost)
  unset(SKIP_INSTALL)
else()
  message( FATAL_ERROR "XGBoost package must export learner-inl.hpp header" )
  # TODO: missing learner-inl.hpp
  # ...my_file.cpp:27:10: fatal error: 'src/learner/learner-inl.hpp' file not found
  hunter_add_package(xgboost)
  find_package(xgboost CONFIG REQUIRED)
  set(XGBOOST_LIB xgboost::xgboost)
endif()

#print_env_var()
#return()

if(BUILD_OGLES_GPGPU)

  # Support local submodule use or external package:
  option(USE_LOCAL_OGLES_GPGPU "Use local ogles_gpgpu" OFF)
  
  if(USE_LOCAL_OGLES_GPGPU)
    add_definitions(-DUSE_LOCAL_OGLES_GPGPU=1)
    set(SKIP_INSTALL ON CACHE BOOL "Avoid installation of submodule libraries" )
    set(OGLES_GPGPU_BUILD_EXAMPLES OFF CACHE BOOL "Don't build examples")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu/ogles_gpgpu")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu/ogles_gpgpu/common")
    add_subdirectory( "${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu" )
    set(OGLES_GPGPU_LIB ogles_gpgpu)
    unset(SKIP_INSTALL)
    unset(BUILD_EXAMPLES)  
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

#print_env_var()

option(BUILD_EOS "EOS 2D-3D fitting" ON)
if(BUILD_EOS)
  ## Header only eos lib for 3D-to-2D model fitting
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/eos/include")
  include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/eos/3rdparty/glm-0.9.7.0") # TODO: hunter package
endif()

if(BUILD_DEST)
  option(USE_LOCAL_DEST "Use local dest" OFF)
  
  if(USE_LOCAL_DEST)
    add_definitions(-DUSE_LOCAL_DEST=1)
    set(DEST_BUILD_EXAMPLES OFF)
    set(DEST_WITH_OPENCV OFF)
    set(SKIP_INSTALL ON CACHE BOOL "Avoid installation of submodule libraries")
    include_directories("${CMAKE_SOURCE_DIR}/src/3rdparty/dest")
    add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/dest)
    set(DEST_LIB dest)
    unset(SKIP_INSTALL)
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
  Boost::filesystem 
  Boost::system 
  Boost::serialization 
  Boost::iostreams
  cereal::cereal  
  ${OpenCV_LIBS} 
  TIFF::libtiff
  PNG::png
  ZLIB::zlib
  Eigen::eigen
  ${DRISHTI_SDK_3RDPARTY_SRC_LIBS}
  spdlog::spdlog
  )

if(cvmatio_FOUND)
  list(APPEND DRISHTI_SDK_3RDPARTY_LIBS cvmatio::cvmatio)
endif()

# Add the sub-libraries
add_subdirectory(lib)

if(BUILD_QT)
  # Some background: https://github.com/ruslo/hunter/issues/253
  hunter_add_package(Qt COMPONENTS qtmultimedia qtdeclarative)
endif()

# Include *nix style console apps (some will run on iOS and Android)
#add_subdirectory(app)

# (Optional) build unit tests
if(BUILD_TESTS)
  add_subdirectory(tests)
endif()

