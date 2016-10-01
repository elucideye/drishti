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

## 3rd party boost pba
include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty/pba)
add_library(boost-pba STATIC ${BOOST_PBA_SRCS})
target_link_libraries(boost-pba PUBLIC Boost::boost)

# Note: Some CMake commands, such as set_property() can't be called on ALIAS targets,
# so we collect SRC and PKG libs separately.

######################################
### Initial 3RDPARTY SRC libraries ###
######################################
set(DRISHTI_SDK_3RDPARTY_SRC_LIBS boost-pba)

######################################
### Initial 3RDPARTY PKG libraries ###
######################################
set(DRISHTI_SDK_3RDPARTY_PKG_LIBS
  dlib::dlib
  ${DRISHTI_SDK_BOOST_LIBS}
  ${OpenCV_LIBS}  
  cereal::cereal
  Eigen::eigen
  spdlog::spdlog
  )

if(DRISHTI_BUILD_FACE)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS cvmatio::cvmatio)
endif()

###############
### XGBOOST ###
###############
option(DRISHTI_USE_LOCAL_XGBOOST "Use local xgboost" OFF)
if(DRISHTI_USE_LOCAL_XGBOOST)
  # Note: Use either XGBOOST
  option(XGBOOST_DO_LEAN "Build lean library for evaluation only" ON)
  option(SKIP_INSTALL "Avoid installation of submodule libraries" ON)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost)
  unset(SKIP_INSTALL CACHE)
  add_library(xgboost::xgboost ALIAS xgboost)
  list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS xgboost::xgboost) # Add library
  set_property(TARGET xgboost PROPERTY FOLDER "libs/3rdparty")  
else()
  hunter_add_package(xgboost)
  find_package(xgboost CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS xgboost::xgboost) # Add library
endif()

###################
### OGLES_GPGPU ###
###################
if(DRISHTI_BUILD_OGLES_GPGPU)
  # Support local submodule use or external package:
  option(DRISHTI_USE_LOCAL_OGLES_GPGPU "Use local ogles_gpgpu" ON)

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
    add_library(ogles_gpgpu::ogles_gpgpu ALIAS ogles_gpgpu)
    list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS ogles_gpgpu::ogles_gpgpu) # Add library
    set_property(TARGET xgboost PROPERTY FOLDER "libs/3rdparty")      
  else()
    add_definitions(-DUSE_LOCAL_OGLES_GPGPU=0)
    hunter_add_package(ogles_gpgpu)
    find_package(ogles_gpgpu CONFIG REQUIRED)
    include_directories("${OGLES_GPGPU_ROOT}/include/ogles_gpgpu")
    include_directories("${OGLES_GPGPU_ROOT}/include/ogles_gpgpu/common")
    list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ogles_gpgpu::ogles_gpgpu) # Add library    
  endif()
  set(OGLES_GPGPU_LIB ogles_gpgpu::ogles_gpgpu)
endif()

# Experimental functionality:
if(DRISHTI_BUILD_EOS)
  hunter_add_package(eos)
  find_package(eos CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS eos::eos) # Add library
  set(EOS_LIB eos::eos)
endif()

# Experimental functionality:
if(DRISHTI_BUILD_DEST)
  hunter_add_package(dest)
  find_package(dest CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS dest::dest) # Add library        
endif()

include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty)

set(DRISHTI_SDK_3RDPARTY_LIBS
  ${DRISHTI_SDK_3RDPARTY_SRC_LIBS}
  ${DRISHTI_SDK_3RDPARTY_PKG_LIBS}
  )

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

##########################
### install: boost-pba ###
##########################

# This must be in the "${drishti_targets_export_name}" exports list for static builds
if(NOT DRISHTI_BUILD_SHARED_SDK)
  message("boost-pba install")
  install(
    TARGETS boost-pba
    EXPORT "${drishti_targets_export_name}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${drishti_include_install_dir}"
    )
endif()
