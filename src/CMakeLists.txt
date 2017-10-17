# Copyright (c) 2014-2017, David Hirvonen
# All rights reserved.

##########################################
### Initial 3RDPARTY SRC/PKG libraries ###
##########################################

# Initialize default lists and populate as we go

set(DRISHTI_SDK_3RDPARTY_PKG_LIBS)
set(DRISHTI_SDK_3RDPARTY_SRC_LIBS)

set(LIB_TYPE STATIC)

################
### 3rdparty ###
################

sugar_include(${DRISHTI_3RD_PARTY_DIR})

##########
### Qt ###
##########

option(DRISHTI_BUILD_QT "Build with Qt" OFF)

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

message("OpenCV_INCLUDE_DIRS = ${OpenCV_INCLUDE_DIRS}")
message("OpenCV_LIB_DIR = ${OpenCV_LIB_DIR}")
message("OpenCV_LIBRARIES = ${OpenCV_LIBRARIES}")
message("OpenCV_LIB_COMPONENTS = ${OpenCV_LIB_COMPONENTS}")
message("OpenCV_LIBS = ${OpenCV_LIBS}")
message("${CMAKE_MODULE_PATH}")


#############
### Beast ### :: optional :: (header only)
#############

option(DRISHTI_USE_BEAST "Use beast (boost) websocket lib" OFF)
if(DRISHTI_USE_BEAST)
  hunter_add_package(Beast)
  find_package(Beast CONFIG REQUIRED)
  # !!! Limit use to applicaiton layer + callbacks, don't add to core libraries !!!!
endif()

##############
### spdlog ###
##############

hunter_add_package(spdlog)
find_package(spdlog CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS spdlog::spdlog)

##############
### cereal ### (header only)
##############

hunter_add_package(cereal)
find_package(cereal CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS cereal::cereal)

############
### dlib ### (header only)
############

hunter_add_package(dlib)
find_package(dlib CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS dlib::dlib)

###########
### glm ### (header only)
###########

hunter_add_package(glm)
find_package(glm CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS glm)

############
### half ### (header only)
############

hunter_add_package(half)
find_package(half CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS half::half)

##############
### Eigen3 ### (header only)
##############

hunter_add_package(Eigen)
find_package(Eigen3 CONFIG REQUIRED)
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

#######################
### thread-pool-cpp ### (header only)
#######################

drishti_option(DRISHTI_USE_THREAD_POOL_CPP "Use thread-pool-cpp submodule" ON IF(DRISHTI_HAVE_THREAD_LOCAL_STORAGE))

if(DRISHTI_USE_THREAD_POOL_CPP)
  hunter_add_package(thread-pool-cpp)
  find_package(thread-pool-cpp CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS thread-pool-cpp::thread-pool-cpp) # Add library
endif()

###############
### xgboost ###
###############

hunter_add_package(xgboost)
find_package(xgboost CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS xgboost::xgboost) # Add library

###################
### ogles_gpgpu ###
###################

option(DRISHTI_BUILD_OGLES_GPGPU "Build with OGLES_GPGPU" ON)

if(DRISHTI_BUILD_OGLES_GPGPU)
  hunter_add_package(ogles_gpgpu)
  find_package(ogles_gpgpu CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ogles_gpgpu::ogles_gpgpu) # Add library
  set(OGLES_GPGPU_LIB ogles_gpgpu::ogles_gpgpu)
endif()

##################
### NEON_2_SSE ### (header only)
##################

hunter_add_package(ARM_NEON_2_x86_SSE)
find_package(ARM_NEON_2_x86_SSE CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ARM_NEON_2_x86_SSE::ARM_NEON_2_x86_SSE)

################
### sse2neon ### (header only)
#################

hunter_add_package(sse2neon)
find_package(sse2neon CONFIG REQUIRED)
list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS sse2neon::sse2neon) # Add library

# We use aglet for an "off screen" OpenGL context.
# This currently provides 3 options:
#   * GLFWContext for most desktop systems (can provide Window)
#   * GLContextIOS
#   * GLContextAndroid
# Note: Android emulator supports GPU use
if(DRISHTI_DO_GPU_TESTING)
  hunter_add_package(aglet)
  find_package(aglet CONFIG REQUIRED)
endif()

###########
### eos ### :: experimental :: (header only)
###########

set(EOS_LIB "") # Make EOS_LIB optional
if(DRISHTI_BUILD_EOS)

  hunter_add_package(Boost COMPONENTS filesystem system)
  find_package(Boost CONFIG REQUIRED system filesystem) # TODO: add upstream to eos/cmake/Config.cmake.in
  
  hunter_add_package(eos)
  find_package(eos CONFIG REQUIRED)
  set(EOS_LIB eos::eos Boost::filesystem)  
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS ${EOS_LIB}) # Add library

endif()

############
### dest ### :: experimental ::
############

if(DRISHTI_BUILD_DEST)
  hunter_add_package(dest)
  find_package(dest CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS dest::dest) # Add library
endif()

############
## imshow ## :: experimental ::
############

drishti_option(DRISHTI_USE_IMSHOW "Use imshow library for local packages" ON IF NOT(ANDROID OR IOS OR MSVC))

if(DRISHTI_USE_IMSHOW)
  hunter_add_package(imshow)
  find_package(imshow CONFIG REQUIRED)
  list(APPEND DRISHTI_SDK_3RDPARTY_PKG_LIBS imshow::imshow) # Add library
endif()

set(DRISHTI_SDK_3RDPARTY_LIBS
  ${DRISHTI_SDK_3RDPARTY_SRC_LIBS}
  ${DRISHTI_SDK_3RDPARTY_PKG_LIBS}
  )

include(sugar_include)
sugar_include(.)

# Add the sub-libraries
add_subdirectory(lib)

###############
### cxxopts ### :: std::regex >= gcc 4.8
###############

hunter_add_package(cxxopts)
find_package(cxxopts CONFIG REQUIRED)

# Include *nix style console apps (some will run on iOS and Android)
if(DRISHTI_BUILD_EXAMPLES)
  add_subdirectory(app)
endif()

# (Optional) build unit tests
if(DRISHTI_BUILD_TESTS)
  add_subdirectory(tests)
  #add_subdirectory(benchmarks)
endif()
