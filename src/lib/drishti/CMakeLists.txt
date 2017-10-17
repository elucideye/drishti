# Copyright (c) 2014-2017, David Hirvonen
# All rights reserved.

include(drishti_symbol_list)
include(drishti_hide)
include(drishti_strip)
include(drishti_split_debug_symbols)

## Customize linker flags
include(CheckCCompilerFlag)
if(NOT MSVC)
  check_c_compiler_flag("-Wl,-dead_strip" FLAG_dead_strip)
  if(NOT FLAG_dead_strip)
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,--gc-sections")
  else()
    set(CMAKE_SHARED_LINKER_FLAGS "${CMAKE_SHARED_LINKER_FLAGS} -Wl,-dead_strip")
  endif()
endif()

set(LIB_TYPE STATIC)

##################
#### world #######
##################

if(DRISHTI_BUILD_WORLD)

  set(DRISHTI_WORLD_SOURCES
    ${DRISHTI_CORE_SRCS}
    ${DRISHTI_CORE_HDRS_PUBLIC}
    ${DRISHTI_GEOMETRY_SRCS}
    ${DRISHTI_GEOMETRY_HDRS_PUBLIC}
    ${DRISHTI_SENSOR_SRCS}
    ${DRISHTI_SENSOR_HDRS_PUBLIC}
    ${DRISHTI_ML_SRCS}
    ${DRISHTI_ML_HDRS_PUBLIC}
    ${DRISHTI_RCPR_SRCS}
    ${DRISHTI_RCPR_HDRS_PUBLIC}
    ${DRISHTI_EYE_SRCS}
    ${DRISHTI_EYE_HDRS_PUBLIC}
    ${DRISHTI_GRAPHICS_SRCS}
    ${DRISHTI_GRAPHICS_HDRS_PUBLIC}
    )

  set_property(GLOBAL PROPERTY USE_FOLDERS ON)

  # Here we provide our own IDE source groupings according to namespace
  # Each source_group() will be a subfolder of the drishti_world target:

  ### core
  source_group("core\\Header Files" FILES ${DRISHTI_CORE_HDRS_PUBLIC})
  source_group("core\\Source Files" FILES ${DRISHTI_CORE_SRCS})

  ### geometry
  source_group("geometry\\Header Files" FILES ${DRISHTI_GEOMETRY_HDRS_PUBLIC})
  source_group("geometry\\Source Files" FILES ${DRISHTI_GEOMETRY_SRCS})

  ### sensor
  source_group("sensor\\Header Files" FILES ${DRISHTI_SENSOR_HDRS_PUBLIC})
  source_group("sensor\\Source Files" FILES ${DRISHTI_SENSOR_SRCS})

  ### ml
  source_group("ml\\Header Files" FILES ${DRISHTI_ML_HDRS_PUBLIC})
  source_group("ml\\Source Files" FILES ${DRISHTI_ML_SRCS})

  ### rcpr
  source_group("rcpr\\Header Files" FILES ${DRISHTI_RCPR_HDRS_PUBLIC})
  source_group("rcpr\\Source Files" FILES ${DRISHTI_RCPR_SRCS})

  ### eye
  source_group("eye\\Header Files" FILES ${DRISHTI_EYE_HDRS_PUBLIC})
  source_group("eye\\Source Files" FILES ${DRISHTI_EYE_SRCS})

  ### graphics
  source_group("graphics\\Header Files" FILES ${DRISHTI_GRAPHICS_HDRS_PUBLIC})
  source_group("graphics\\Source Files" FILES ${DRISHTI_GRAPHICS_SRCS})

  if(DRISHTI_BUILD_ACF)
    ### acf
    source_group("acf\\Header Files" FILES ${DRISHTI_ACF_HDRS_PUBLIC})
    source_group("acf\\Source Files" FILES ${DRISHTI_ACF_SRCS})
    list(APPEND DRISHTI_WORLD_SOURCES ${DRISHTI_ACF_SRCS} ${DRISHTI_ACF_HDRS_PUBLIC})
  endif()

  if(DRISHTI_BUILD_FACE)
    ### face
    source_group("face\\Header Files" FILES ${DRISHTI_FACE_HDRS_PUBLIC})
    source_group("face\\Source Files" FILES ${DRISHTI_FACE_SRCS})
    list(APPEND DRISHTI_WORLD_SOURCES ${DRISHTI_FACE_SRCS} ${DRISHTI_FACE_HDRS_PUBLIC})
  endif()

  if(DRISHTI_BUILD_OGLES_GPGPU)
    list(APPEND DRISHTI_SDK_3RDPARTY_LIBS ${OGLES_GPGPU_LIB})

    ### hci
    # currently only works w/ ogles_gpgpu
    # TODO: needs CPU only path
    if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU)
      source_group("hci\\Header Files" FILES ${DRISHTI_HCI_HDRS_PUBLIC})
      source_group("hci\\Source Files" FILES ${DRISHTI_HCI_SRCS})
      list(APPEND DRISHTI_WORLD_SOURCES ${DRISHTI_HCI_SRCS} ${DRISHTI_HCI_HDRS_PUBLIC})
    endif()

  endif()

  add_library(drishti_world ${LIB_TYPE} ${DRISHTI_WORLD_SOURCES})
  set_property(TARGET drishti_world PROPERTY FOLDER "libs/drishti")
  drishti_hide(drishti_world)

  target_link_libraries(drishti_world PUBLIC ${DRISHTI_SDK_3RDPARTY_LIBS} xgboost::xgboost Eigen3::Eigen)

  if(DRISHTI_USE_THREAD_POOL_CPP)
    target_link_libraries(drishti_world PUBLIC thread-pool-cpp::thread-pool-cpp)
  endif()

  if(DRISHTI_COTIRE)
    cotire(drishti_world)
    set(drishti_libs drishti_world_unity)
    set(drishti_regression_libs drishti_world_unity)
    set(drishti_half_libs drishti_world_unity)
  else()
    set(drishti_libs drishti_world)
    set(drishti_regression_libs drishti_world)
    set(drishti_half_libs drishti_world)
  endif()
else()

  ## drishti_core
  add_library(drishti_core ${LIB_TYPE} ${DRISHTI_CORE_SRCS} ${DRISHTI_CORE_HDRS_PUBLIC})
  if(NOT TARGET Eigen3::Eigen)
    message(FATAL_ERROR "Not a target")
  endif()
  target_link_libraries(drishti_core PUBLIC ${OpenCV_LIBS} Eigen3::Eigen)
  if(DRISHTI_USE_THREAD_POOL_CPP)
    target_link_libraries(drishti_core PUBLIC thread-pool-cpp::thread-pool-cpp)
  endif()

  ## drishti_geometry
  add_library(drishti_geometry ${LIB_TYPE} ${DRISHTI_GEOMETRY_SRCS} ${DRISHTI_GEOMETRY_HDRS_PUBLIC})
  target_link_libraries(drishti_geometry PUBLIC ${OpenCV_LIBS})

  ## drishti_sensors
  add_library(drishti_sensor ${LIB_TYPE} ${DRISHTI_SENSOR_SRCS} ${DRISHTI_SENSOR_HDRS_PUBLIC})
  target_link_libraries(drishti_sensor PUBLIC ${OpenCV_LIBS})

  ## drishti_ml
  add_library(drishti_ml ${LIB_TYPE} ${DRISHTI_ML_SRCS} ${DRISHTI_ML_HDRS_PUBLIC})
  target_link_libraries(drishti_ml PUBLIC ${OpenCV_LIBS} xgboost::xgboost)

  ## drishti_rcpr
  add_library(drishti_rcpr ${LIB_TYPE} ${DRISHTI_RCPR_SRCS} ${DRISHTI_RCPR_HDRS_PUBLIC})
  target_link_libraries(drishti_rcpr PUBLIC drishti_ml ${OpenCV_LIBS} xgboost::xgboost)

  ## drishti_eye
  add_library(drishti_eye ${LIB_TYPE} ${DRISHTI_EYE_SRCS} ${DRISHTI_EYE_HDRS_PUBLIC})
  target_link_libraries(drishti_eye PUBLIC drishti_ml drishti_rcpr drishti_geometry ${OpenCV_LIBS} xgboost::xgboost)

  ## drishti_graphics
  if(DRISHTI_GRAPHICS_SRCS)
    add_library(drishti_graphics ${LIB_TYPE} ${DRISHTI_EYE_SRCS} ${DRISHTI_EYE_HDRS_PUBLIC})
    if(DRISHTI_BUILD_OGLES_GPGPU)
      target_link_libraries(drishti_graphics PUBLIC ${OGLES_GPGPU_LIB} glm)
    endif()
  endif()

  ## drishti_acf
  if(DRISHTI_BUILD_ACF)
    add_library(drishti_acf ${LIB_TYPE} ${DRISHTI_ACF_SRCS} ${DRISHTI_ACF_HDRS_PUBLIC})
    if(DRISHTI_BUILD_OGLES_GPGPU)
      target_link_libraries(drishti_acf PUBLIC ${OGLES_GPGPU_LIB} ${OpenCV_LIBS})
    endif()
    set(DRISHTI_OBJ_ACF drishti_acf)
  endif()

  ## drishti_face
  if(DRISHTI_BUILD_FACE)
    add_library(drishti_face ${LIB_TYPE} ${DRISHTI_FACE_SRCS} ${DRISHTI_FACE_HDRS_PUBLIC})
    target_link_libraries(drishti_face PUBLIC drishti_eye ${DRISHTI_OBJ_ACF} ${OpenCV_LIBS} ${EOS_LIB})
    set(DRISHTI_OBJ_FACE drishti_face)
  endif()

  if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU)
    add_library(drishti_hci ${LIB_TYPE} ${DRISHTI_HCI_SRCS} ${DRISHTI_HCI_NDRS_PUBLIC})
    target_link_libraries(drishti_hci PUBLIC ${DRISHTI_OBJ_FACE} ${DRISHTI_OBJ_ACF} drishti_eye ${OpenCV_LIBS} ${EOS_LIB})
    set(DRISHTI_OBJ_HCI drishti_hci)
  endif()

  # Define a list of individual static libraries
  set(drishti_libs
    ${DRISHTI_OBJ_HCI}
    ${DRISHIT_OBJ_ACF}
    ${DRISHTI_OBJ_FACE}
    drishti_eye
    drishti_rcpr
    drishti_ml
    drishti_geometry
    drishti_sensor
    drishti_core
    )

  set(drishti_half_libs drishti_ml drishti_rcpr drishti_acf)
  set(drishti_regression_libs drishti_ml) # for DRISHTI_BUILD_REGRESSION_*

  foreach(library ${drishti_libs})
    target_link_libraries(${library} PUBLIC Eigen3::Eigen)
    drishti_hide(${library})
  endforeach()

endif(DRISHTI_BUILD_WORLD)

# For Android platforms, we must specify -DHALF_ENABLE_CPP11_CMATH=0 for all modules
# using the half precision floating point storage (i.e., package half)
if(ANDROID)
  foreach(library ${drishti_half_libs})
    target_compile_definitions(${library} PUBLIC HALF_ENABLE_CPP11_CMATH=0)
  endforeach()
endif()

# For all regression modules, we must provide definitions corresponding
# to options for ensemble of regression tree leaf node accumulation:
#   -DDRISHTI_BUILD_REGRESSION_SIMD=(0|1)
#   -DDRISHTI_BUILD_REGRESSION_FIXED_POINT=(0|1)
foreach(library ${drishti_regression_libs})
  drishti_bool_to_int(DRISHTI_BUILD_REGRESSION_SIMD build_regression_simd)
  target_compile_definitions(${library} PUBLIC DRISHTI_BUILD_REGRESSION_SIMD=${build_regression_simd})
  drishti_bool_to_int(DRISHTI_BUILD_REGRESSION_FIXED_POINT build_regression_fixed_point)
  target_compile_definitions(${library} PUBLIC DRISHTI_BUILD_REGRESSION_FIXED_POINT=${build_regression_fixed_point})
endforeach()

# Build and install a single library or framework from our set of "object" libraries
# Note: Due to complications with object libraries we are either using per module
# static libraries or a single drishti_world compilation from all global siources
# gathered by the Sugar package.
#   * https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=15038&graph=dependency
#   * http://www.cmake.org/pipermail/cmake/2014-February/057055.html
#   * https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=14970&graph=relation
set(drishti_sdk_srcs master/drishti_master.hpp master/drishti_master.cpp)

# Create dependency list
set(drishti_sdk_libs ${drishti_libs} ${DRISHTI_SDK_3RDPARTY_LIBS})

if(${DRISHTI_DO_GPU_TESTING} AND MSVC)
  # FaceTracker.cpp -> FaceTracker.hpp -> Context.hpp -> drishti_gl.hpp -> gl/glew.h
  hunter_add_package(glew)
  find_package(glew CONFIG REQUIRED)
endif()

### drishtisdk (static)
add_library(drishtisdk ${drishti_sdk_srcs})
target_link_libraries(drishtisdk PUBLIC ${drishti_sdk_libs})
drishti_strip(drishtisdk)
drishti_hide(drishtisdk)
set_target_properties(drishtisdk
  PROPERTIES
  PUBLIC_HEADER "${DRISHTISDK_PUBLIC_HDRS}"
  )

if(NOT IOS)
  set_target_properties(drishtisdk
    PROPERTIES
    SOVERSION "${drishtisdk_VERSION_MAJOR}"
    VERSION "${drishtisdk_VERSION}"
    )
endif()

# Set the serialization mode preprocessor flags (boost and/or cereal)
set(DRISHTI_LIBRARY_LIST ${drishti_libs} drishtisdk)
foreach(library ${DRISHTI_LIBRARY_LIST})

  target_include_directories(${library}
    PUBLIC
    "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>"
    "$<INSTALL_INTERFACE:${drishti_include_install_dir}>"
    )

  if(DRISHTI_OPENGL_ES3)
    target_compile_definitions(${library} PUBLIC DRISHTI_OPENGL_ES3=1)
  endif()

  # -DDRISHTI_BUILD_MIN_SIZE=(0|1)
  drishti_bool_to_int(DRISHTI_BUILD_MIN_SIZE build_min_size)
  target_compile_definitions(${library} PUBLIC DRISHTI_BUILD_MIN_SIZE=${build_min_size})

  message("DRISHTI_BUILD_MIN_SIZE=${build_min_size} ${DRISHTI_BUILD_MIN_SIZE}")

  # define M_PI_2 for MSVC
  target_compile_definitions(${library} PUBLIC _USE_MATH_DEFINES)

  if(DRISHTI_SERIALIZE_WITH_CVMATIO)
    target_compile_definitions(${library} PUBLIC DRISHTI_SERIALIZE_WITH_CVMATIO=1)
  endif()

  if(DRISHTI_BUILD_OGLES_GPGPU)
    target_compile_definitions(${library} PUBLIC DRISHTI_BUILD_OGLES_GPGPU=1)
  endif()

  if(DRISHTI_BUILD_QT)
    target_compile_definitions(${library} PUBLIC DRISHTI_BUILD_QT=1)
  endif()

endforeach()

############################
###### PUBLIC API ##########
############################

if(DRISHTI_COPY_3RDPARTY_LICENSES)
  # Install license files
  include(drishti_copy_3rdparty_licenses)
  drishti_copy_3rdparty_licenses("3rdparty/licenses")
endif()

#################################################
### Generate the main shared public interface ###
#################################################

# keep header file version strings up to date:
set(drishti_sdk_hpp_in "${CMAKE_CURRENT_LIST_DIR}/drishti/drishti_sdk.hpp.in")
set(drishti_sdk_hpp "${CMAKE_CURRENT_BINARY_DIR}/drishti/drishti_sdk.hpp")
configure_file("${drishti_sdk_hpp_in}" "${drishti_sdk_hpp}" @ONLY)

# The public interface can be built as either SHARED or STATIC.
# There are two classes of end users: 1) hunter and 2) non-hunter.
# For hunter users, either SHARED OR STATIC targets can be used
# transparently.  For non-hunter users, SHARED libraries will
# provide dependency free use automatically, or the
# DRISHTI_MERGE_ALL_LIBS option can be used to provide a single
# static library (all dependencies are identified, split and
# merged into a single monolithic library for convenient dependency
# free linking).

if(DRISHTI_BUILD_SHARED_SDK)
  set(api_lib_type SHARED)
else()
  set(api_lib_type STATIC)
endif()

if((NOT DRISHTI_BUILD_SHARED_SDK) AND DRISHTI_BUILD_MERGED_SDK)
  set(drishti_api_name drishti_stage) # create a temporary target
else()
  set(drishti_api_name drishti)
endif()

# These targets will be configured with appropriate include paths:
set(drishti_interface_libs "")

# Create the public shared library
add_library(${drishti_api_name} ${api_lib_type}
  ${DRISHTI_DRISHTI_SRCS}
  ${DRISHTI_DRISHTI_HDRS_PUBLIC}
  ${DRISHTI_DRISHTI_HDRS_PRIVATE}
  ${drishti_sdk_hpp})
target_compile_definitions(${drishti_api_name} PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
target_link_libraries(${drishti_api_name} PRIVATE "${drishti_sdk_libs}")
list(APPEND drishti_interface_libs ${drishti_api_name})

if((NOT DRISHTI_BUILD_SHARED_SDK) AND DRISHTI_BUILD_MERGED_SDK)
  # (Optional) merge top level library and all dependencies:
  # Also see: http://stackoverflow.com/a/18949281
  include(drishti_merge_libraries)
  drishti_merge_libraries(LIBRARIES ${drishti_api_name} FINAL drishti)
  list(APPEND drishti_interface_libs drishti)
endif()

drishti_hide(drishti)

if(IOS AND DRISHTI_BUILD_MERGED_SDK)
  # Error: ld: Assertion failed: (0 && "need to handle arm64 -r reloc")
else()
  drishti_strip(drishti)
endif()

if(DRISHTI_BUILD_SHARED_SDK)  
  drishti_split_debug_symbols(drishti)  
endif()
  
##############################################################
### Generate export header and set build/install include paths
##############################################################

include (GenerateExportHeader)
set(drishti_export_header "${CMAKE_CURRENT_BINARY_DIR}/drishti/drishti_export.hpp")
generate_export_header(drishti
  EXPORT_FILE_NAME "${drishti_export_header}"
  )

set_target_properties(drishti
  PROPERTIES
  PUBLIC_HEADER "${DRISHTI_DRISHTI_HDRS_PUBLIC}"
  )

if(IOS)
  # This is required for some Xcode archive operations in end users of a shared framework
  set_target_properties(drishti
    PROPERTIES
    XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
    )
endif()

if(NOT IOS)
  # Use standrad shared lib versioning except for Xcode shared frameworks
  set_target_properties(drishti
    PROPERTIES
    VERSION "${drishtisdk_VERSION}"
    SOVERSION "${drishtisdk_VERSION_MAJOR}"
    )
endif()

foreach(library ${drishti_libs} drishti drishtisdk)
  set_property(TARGET ${library} PROPERTY FOLDER "libs/drishti")
endforeach()

###################
##### install #####
###################

if(DRISHTI_INSTALL)

  # See: https://cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file
  if(DRISHTI_BUILD_SHARED_SDK)
    set(DRISHTI_INSTALL_TARGETS "")
  else()
    set(DRISHTI_INSTALL_TARGETS ${drishti_libs})
  endif()

  # Export minimal public SDK or full internal SDK
  set(TARGET_SDK "drishti")
  set(TARGET_PUBLIC_HDRS ${DRISHTI_DRISHTI_HDRS_PUBLIC} "${drishti_export_header}" "${drishti_sdk_hpp}")

  include(CMakePackageConfigHelpers)
  write_basic_package_version_file("${drishti_version_config}"
    VERSION "${drishtisdk_VERSION}"
    COMPATIBILITY SameMajorVersion
    )

  # Note: variable 'drishti_targets_export_name' used
  configure_file("${PROJECT_SOURCE_DIR}/cmake/Config.cmake.in" "${drishti_project_config}" @ONLY)

  install(
    TARGETS ${TARGET_SDK} ${DRISHTI_INSTALL_TARGETS}
    EXPORT "${drishti_targets_export_name}"
    LIBRARY DESTINATION "lib"
    ARCHIVE DESTINATION "lib"
    RUNTIME DESTINATION "bin"
    INCLUDES DESTINATION "${drishti_include_install_dir}"
    PUBLIC_HEADER DESTINATION "${drishti_include_install_dir}/${CMAKE_PROJECT_NAME}"
    )

  install(
    FILES
    ${TARGET_PUBLIC_HDRS}
    DESTINATION "${drishti_include_install_dir}/${CMAKE_PROJECT_NAME}"
    )

  install(
    FILES "${drishti_project_config}" "${drishti_version_config}"
    DESTINATION "${drishti_config_install_dir}"
    )

  install(
    EXPORT "${drishti_targets_export_name}"
    NAMESPACE "${drishti_namespace}"
    DESTINATION "${drishti_config_install_dir}"
    )  
endif()

# Experimental size reduction

##################
##### C API ######
##################

if(DRISHTI_BUILD_C_INTERFACE)

  if((NOT DRISHTI_BUILD_SHARED_SDK) AND DRISHTI_BUILD_MERGED_SDK)
    set(drishti_c_api_name drishti_c_stage) # create a temporary target
  else()
    set(drishti_c_api_name drishti_c)
  endif()

  add_library(${drishti_c_api_name} ${api_lib_type} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISHTI_HDRS_PUBLIC})
  set_property(TARGET ${drishti_c_api_name} PROPERTY FOLDER "libs/drishti")
  target_compile_definitions(${drishti_c_api_name} PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
  target_link_libraries(${drishti_c_api_name} PRIVATE "${drishti_sdk_libs}")
  list(APPEND drishti_interface_libs ${drishti_c_api_name})

  if((NOT DRISHTI_BUILD_SHARED_SDK) AND DRISHTI_BUILD_MERGED_SDK)
    # (Optional) merge top level library and all dependencies:
    # Also see: http://stackoverflow.com/a/18949281
    include(drishti_merge_libraries)
    drishti_merge_libraries(LIBRARIES ${drishti_c_api_name} FINAL drishti_c)
    list(APPEND drishti_interface_libs drishti_c)
  else()
    # Only use symbol_list and version script for non drishti_merge_libraries scenarios:
    # Undefined symbols for architecture x86_64:
    # "_drishti_eye_segmenter_create_from_file", referenced from:
    drishti_symbol_list(drishti_c)
  endif()
  drishti_hide(drishti_c)

  if(IOS AND DRISHTI_BUILD_MERGED_SDK)
    # Error: ld: Assertion failed: (0 && "need to handle arm64 -r reloc")
  else()
    drishti_strip(drishti_c)
  endif()

  if(DRISHTI_INSTALL)  
    # Simple install to execute post build strip
    install(
      TARGETS drishti_c
      EXPORT "${drishti_targets_export_name}"
      LIBRARY DESTINATION "lib"
      ARCHIVE DESTINATION "lib"
      RUNTIME DESTINATION "bin"
      INCLUDES DESTINATION "${drishti_include_install_dir}"
      )
  endif()  
endif()

foreach(library ${drishti_interface_libs})
  target_include_directories(${library}
    PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_SOURCE_DIR}>"
    "$<INSTALL_INTERFACE:include>"  # <prefix>/include
    )
endforeach()

#################
## Unit tests ###
#################

# NOTE: These should come last to ensure all library targets are available
if(DRISHTI_BUILD_TESTS)

  if(IOS)
    # Add a static public SDK library:
    # * support CTest/GTest executables on iOS until CMake dynamic framework embedding is sorted out
    # * use this library for IDE debugging as an alternative to the stripped public library
    add_library(drishti_static STATIC ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISHTI_HDRS_PUBLIC})
    target_compile_definitions(drishti_static PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
    target_link_libraries(drishti_static PRIVATE "${drishti_sdk_libs}")
    set_property(TARGET drishti_static PROPERTY FOLDER "libs/drishti")
    target_include_directories(drishti_static
      PUBLIC
      "$<BUILD_INTERFACE:${CMAKE_CURRENT_BINARY_DIR};${CMAKE_CURRENT_SOURCE_DIR}>"
      "$<INSTALL_INTERFACE:include>"  # <prefix>/include
      )
  endif()
  
endif()
