##########################################
### Initial 3RDPARTY SRC/PKG libraries ###
##########################################

# Initialize default lists and populate as we go
set(DRISHTI_SDK_3RDPARTY_SRC_LIBS)
set(DRISHTI_SDK_3RDPARTY_PKG_LIBS)

set(LIB_TYPE STATIC)

################
### 3rdparty ###
################

sugar_include(${CMAKE_SOURCE_DIR}/src/3rdparty)

##########
### Qt ###
##########

option(DRISHTI_BUILD_QT "Build with Qt" OFF)
drishti_mirror_definition(DRISHTI_BUILD_QT)

if(DRISHTI_BUILD_QT)
  hunter_add_package(Qt COMPONENTS qtmultimedia qtsensors qtquickcontrols)
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

##############
### OpenCV ###
##############

hunter_add_package(OpenCV)
find_package(OpenCV REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ${OpenCV_LIBS})

include_directories("${OpenCV_INCLUDE_DIRS}") # to be deprecated
message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
message("OpenCV_LIB_COMPONENTS = ${OpenCV_LIB_COMPONENTS}")
message("OpenCV_LIBS = ${OpenCV_LIBS}")
message("${CMAKE_MODULE_PATH}")

#############
### Boost ###
#############

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

if(DRISHTI_SERIALIZE_WITH_BOOST)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ${DRISHTI_SDK_BOOST_LIBS})
endif()

##############
### spdlog ###
##############

hunter_add_package(spdlog)
find_package(spdlog CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS spdlog::spdlog)

##############
### cereal ###
##############

hunter_add_package(cereal)
find_package(cereal CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS cereal::cereal)

############
### dlib ###
############

hunter_add_package(dlib)
find_package(dlib CONFIG REQUIRED)
include_directories(${DLIB_INCLUDE_DIRS})
add_definitions(-DDLIB_NO_GUI_SUPPORT=1)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS dlib::dlib)

############
### half ###
############

hunter_add_package(half)
find_package(half CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS half::half)

##############
### Eigen3 ###
##############

hunter_add_package(Eigen)
find_package(Eigen3 CONFIG REQUIRED) # <== not finding hunter provided eigen
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS Eigen3::Eigen)

###############
### cvmatio ###
###############

if(DRISHTI_BUILD_ACF)
  if(DRISHTI_SERIALIZE_WITH_CVMATIO)
    hunter_add_package(cvmatio)
    find_package(cvmatio REQUIRED CONFIG)
    list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS cvmatio::cvmatio)
  endif()
endif()

##################
### boost::pba ###
##################

if(DRISHTI_SERIALIZE_WITH_BOOST)
  ## 3rd party boost pba
  add_library(boost-pba STATIC ${BOOST_PBA_SRCS})
  target_link_libraries(boost-pba PUBLIC Boost::boost)
  target_include_directories(boost-pba PUBLIC $<BUILD_INTERFACE:${CMAKE_SOURCE_DIR}/src/3rdparty/pba>)
  set_property(TARGET boost-pba PROPERTY FOLDER "libs/3rdparty")
  add_library(boost-pba::boost-pba ALIAS boost-pba)
  list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS boost-pba::boost-pba)
endif()

# Note: Some CMake commands, such as set_property() can't be called on ALIAS targets.

#######################
### thread-pool-cpp ###
#######################

drishti_option(DRISHTI_USE_THREAD_POOL_CPP "Use thread-pool-cpp submodule" ON IF(DRISHTI_HAVE_THREAD_LOCAL_STORAGE))
option(DRISHTI_USE_LOCAL_THREAD_POOL_CPP "Use thread-pool-cpp submodule" ON)

if(DRISHTI_USE_THREAD_POOL_CPP)
  if(DRISHTI_USE_LOCAL_THREAD_POOL_CPP)
    option(THREAD_POOL_CPP_HAS_CPP14 "Has true C++14 support." OFF) # override option
    option(THREAD_POOL_CPP_BUILD_TESTS "Build tests." OFF)
    option(THREAD_POOL_CPP_BUILD_BENCHMARKS "Build benchmarks." OFF)
    option(THREAD_POOL_CPP_INSTALL "Install library." OFF)
    add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/thread-pool-cpp)
    add_library(thread-pool-cpp::thread-pool-cpp ALIAS thread-pool-cpp)
    list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS thread-pool-cpp::thread-pool-cpp) # Add library
  else()
    hunter_add_package(thread-pool-cpp)
    find_package(thread-pool-cpp CONFIG REQUIRED)
    list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS thread-pool-cpp::thread-pool-cpp) # Add library
  endif()
endif()

###############
### xgboost ###
###############

option(DRISHTI_USE_LOCAL_XGBOOST "Use local xgboost" ON)
if(DRISHTI_USE_LOCAL_XGBOOST)
  option(XGBOOST_DO_LEAN "Build lean library for evaluation only" ON)
  option(SKIP_INSTALL "Avoid installation of submodule libraries" ON)
  add_subdirectory(${CMAKE_SOURCE_DIR}/src/3rdparty/xgboost)
  include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty)
  unset(SKIP_INSTALL CACHE)
  add_library(xgboost::xgboost ALIAS xgboost)
  list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS xgboost::xgboost) # Add library
  set_property(TARGET xgboost PROPERTY FOLDER "libs/3rdparty")
else()
  hunter_add_package(xgboost)
  find_package(xgboost CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS xgboost::xgboost) # Add library
endif()
add_definitions(-DHAS_XGBOOST=1) # legacy

###################
### ogles_gpgpu ###
###################

option(DRISHTI_BUILD_OGLES_GPGPU "Build with OGLES_GPGPU" ON)
drishti_mirror_definition(DRISHTI_BUILD_OGLES_GPGPU)

if(DRISHTI_BUILD_OGLES_GPGPU)

  # Support local submodule use or external package:
  option(DRISHTI_USE_LOCAL_OGLES_GPGPU "Use local ogles_gpgpu" ON)

  if(DRISHTI_USE_LOCAL_OGLES_GPGPU)
    option(OGLES_GPGPU_INSTALL "Avoid installation of submodule libraries" OFF)
    option(OGLES_GPGPU_BUILD_EXAMPLES "Don't build examples" OFF)
    option(OGLES_GPGPU_VERBOSE "Don't be verbose" OFF)
    add_subdirectory( "${CMAKE_SOURCE_DIR}/src/3rdparty/ogles_gpgpu" )
    set(OGLES_GPGPU_LIB ogles_gpgpu)
    add_library(ogles_gpgpu::ogles_gpgpu ALIAS ogles_gpgpu)
    list(APPEND DRISHTI_SDK_3RDPARTY_SRC_LIBS ogles_gpgpu::ogles_gpgpu) # Add library
    set_property(TARGET ogles_gpgpu PROPERTY FOLDER "libs/3rdparty")
  else()
    hunter_add_package(ogles_gpgpu)
    find_package(ogles_gpgpu CONFIG REQUIRED)
    list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ogles_gpgpu::ogles_gpgpu) # Add library
  endif()
  set(OGLES_GPGPU_LIB ogles_gpgpu::ogles_gpgpu)
endif()

############
### simd ###
############

include_directories(${CMAKE_SOURCE_DIR}/src/3rdparty/simd)

###########
### eos ### experimental
###########

set(EOS_LIB "") # Make EOS_LIB optional
if(DRISHTI_BUILD_EOS)
  hunter_add_package(eos)
  find_package(eos CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS eos::eos) # Add library
  set(EOS_LIB eos::eos)
endif()

############
### dest ### experimental
############

if(DRISHTI_BUILD_DEST)
  hunter_add_package(dest)
  find_package(dest CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS dest::dest) # Add library
endif()

# opencv + linux JPEG workaround
if(is_linux)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS JPEG::jpeg)
endif()

set(DRISHTI_SDK_3RDPARTY_LIBS
  ${DRISHTI_SDK_3RDPARTY_SRC_LIBS}
  ${DRISHTI_SDK_3RDPARTY_PKG_LIBS}
  )

# Add the sub-libraries
add_subdirectory(lib)

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
