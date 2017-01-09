include(drishti_symbol_list)
include(drishti_hide)
include(drishti_strip)
include(sugar_include)

sugar_include(.)

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
    ${DRISHTI_CORE_SRCS} ${DRISHTI_CORE_HDRS_PUBLIC}
    ${DRISHTI_GEOMETRY_SRCS} ${DRISHTI_GEOMETRY_HDRS_PUBLIC}
    ${DRISHTI_SENSOR_SRCS} ${DRISHTI_SENSOR_HDRS_PUBLIC}
    ${DRISHTI_ML_SRCS} ${DRISHTI_ML_HDRS_PUBLIC}
    ${DRISHTI_RCPR_SRCS} ${DRISHTI_RCPR_HDRS_PUBLIC}
    ${DRISHTI_EYE_SRCS} ${DRISHTI_EYE_HDRS_PUBLIC}
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
  
  if(DRISHTI_BUILD_ACF)
    ### acf
    source_group("acf\\Header Files" FILES ${DRISHTI_ACF_HDRS_PUBLIC})
    source_group("acf\\Source Files" FILES ${DRISHTI_ACF_SRCS})    
    include_directories(acf/acf/toolbox)
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
  
  target_link_libraries(drishti_world ${DRISHTI_SDK_3RDPARTY_LIBS} xgboost::xgboost Eigen::eigen)

  if(DRISHTI_COTIRE)  
    cotire(drishti_world)
    set(DRISHTI_LIBS drishti_world_unity)
  else()
    set(DRISHTI_LIBS drishti_world)
  endif()

  
else()

  ## drishti_core
  add_library(drishti_core ${LIB_TYPE} ${DRISHTI_CORE_SRCS} ${DRISHTI_CORE_HDRS_PUBLIC})
  target_link_libraries(drishti_core PUBLIC ${OpenCV_LIBS} thread-pool-cpp::thread-pool-cpp)

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

  ## drishti_acf
  if(DRISHTI_BUILD_ACF)
    include_directories(acf/acf/toolbox)
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
  set(DRISHTI_LIBS
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

  foreach(library ${DRISHTI_LIBS})
    target_link_libraries(${library} PUBLIC Eigen::eigen)
    drishti_hide(${library})
  endforeach()  

endif(DRISHTI_BUILD_WORLD)

##
## Unit tests
##

message("DRISHTI_HAS_TEST_TOOLS : ${DRISHTI_HAS_TEST_TOOLS}") 
if(DRISHTI_BUILD_TESTS AND ${DRISHTI_HAS_TEST_TOOLS} AND (DRISHTI_SERIALIZE_WITH_BOOST OR DRISHTI_SERIALIZE_WITH_CEREAL))

  message("DRISHTI_TESTING...")

  add_subdirectory(testlib)

  add_subdirectory(core)
  add_subdirectory(geometry)
  add_subdirectory(ml)

  # Run public SDK, then eye tests in that order
  add_subdirectory(drishti)  
  add_subdirectory(eye)
  
  if(DRISHTI_BUILD_ACF)
    add_subdirectory(acf)
  endif()
  if(DRISHTI_BUILD_FACE)
    add_subdirectory(face)
  endif()
  if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU)
    add_subdirectory(hci)
  endif()
endif()

include_directories("${PROJECT_BINARY_DIR}")

##
## Build and install a single library or framework from our set of object libraries
##

# https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=15038&graph=dependency
# http://www.cmake.org/pipermail/cmake/2014-February/057055.html
# https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=14970&graph=relation
set(DRISHTI_SDK_SRCS master/drishti_master.hpp master/drishti_master.cpp)

# Create dependency list
set(DRISHTI_SDK_LIBS ${DRISHTI_LIBS} ${DRISHTI_SDK_3RDPARTY_LIBS})

######################
####### OpenGL #######
######################

set(DRISHTI_OPENGL_LIBS "")
if(APPLE)
  if(IOS)
    set(DRISHTI_OPENGL_LIBS "-framework OpenGLES")
  else()
    set(DRISHTI_OPENGL_LIBS "-framework OpenGL")
  endif()
elseif(ANDROID)
  set(DRISHTI_OPENGL_LIBS EGL GLESv2)
else()
  find_package(OpenGL REQUIRED)
  set(DRISHTI_OPENGL_LIBS "${OPENGL_LIBRARIES}")
endif()
list(APPEND DRISHTI_SDK_LIBS "${DRISHTI_OPENGL_LIBS}")

if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU)
  if(APPLE)
    # These frameworks are needed for  optimized texture cache in GPGPU build
    # of the final shared library
    if(IOS)
      list(APPEND DRISHTI_SDK_LIBS
        "-framework ImageIO"
        "-framework CoreFoundation"
        "-framework Foundation" # NSLog
        )
    else()
      list(APPEND DRISHTI_SDK_LIBS
        "-framework ImageIO"
        "-framework CoreVideo"    
        )
    endif()
  endif(APPLE)
endif()

### drishtisdk (static)
add_library(drishtisdk ${DRISHTI_SDK_SRCS})
target_link_libraries(drishtisdk PUBLIC ${DRISHTI_SDK_LIBS})
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
set(DRISHTI_LIBRARY_LIST ${DRISHTI_LIBS} drishtisdk)
foreach(library ${DRISHTI_LIBRARY_LIST})

  # define M_PI_2 for MSVC
  target_compile_definitions(${library} PUBLIC _USE_MATH_DEFINES) 

  if(DRISHTI_SERIALIZE_WITH_BOOST)
    target_compile_definitions(${library} PUBLIC DRISHTI_SERIALIZE_WITH_BOOST=1)
    if(DRISHTI_USE_TEXT_ARCHIVES)
      target_compile_definitions(${library} PUBLIC DRISHTI_USE_TEXT_ARCHIVES=1)
    endif()
  endif()

  if(DRISHTI_SERIALIZE_WITH_CEREAL)
    target_compile_definitions(${library} PUBLIC DRISHTI_SERIALIZE_WITH_CEREAL=1)
  endif()

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

string(COMPARE EQUAL "${CMAKE_INSTALL_PREFIX}" "" no_install_prefix)
if(no_install_prefix)
  message(FATAL_ERROR "CMAKE_INSTALL_PREFIX expected")
endif()

set(THIRD_PARTY_INSTALL_DIR "${CMAKE_INSTALL_PREFIX}/3rdparty")
add_custom_command(TARGET drishtisdk POST_BUILD
  COMMAND ${CMAKE_COMMAND} -E make_directory "${THIRD_PARTY_INSTALL_DIR}"
  )
foreach(library ${DRISHTI_SDK_3RDPARTY_LIBS})
  if(TARGET ${library}  ## Avoid generator expression errors for header only libs
      AND NOT "${library}" MATCHES "cereal::cereal"
      AND NOT "${library}" MATCHES "dlib::dlib"
      AND NOT "${library}" MATCHES "eos::eos"
      AND NOT "${library}" MATCHES "spdlog::spdlog"
      AND NOT "${library}" MATCHES "Eigen::eigen"
      AND NOT "${library}" MATCHES "thread-pool-cpp::thread-pool-cpp")
    add_custom_command(TARGET drishtisdk POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${library}> "${THIRD_PARTY_INSTALL_DIR}"
      COMMENT "Copying " $<TARGET_FILE:${library}>
      )
  endif()
endforeach()

##
## Install license files:
##

include(drishti_copy_3rdparty_licenses)
drishti_copy_3rdparty_licenses("${THIRD_PARTY_INSTALL_DIR}/licenses")

if(DRISHTI_BUILD_SHARED_SDK)
  set(API_LIB_TYPE SHARED)
else()
  set(API_LIB_TYPE STATIC)
endif()

## Always static for ios (we'll build dynamic framework from within xcode)

# Create the public shared library
add_library(drishti ${API_LIB_TYPE} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISHTI_HDRS_PUBLIC})
target_compile_definitions(drishti PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
target_link_libraries(drishti PRIVATE "${DRISHTI_SDK_LIBS}")
drishti_hide(drishti)
drishti_strip(drishti)

if(IOS)
  # Add a static public SDK library for initial iOS testing:
  add_library(drishti_static STATIC ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISHTI_HDRS_PUBLIC})
  target_compile_definitions(drishti_static PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
  target_link_libraries(drishti_static PRIVATE "${DRISHTI_SDK_LIBS}")
endif()

set_target_properties(drishti
  PROPERTIES
  PUBLIC_HEADER "${DRISHTI_DRISHTI_HDRS_PUBLIC}"
  )

if(NOT IOS)
  # iOS dynamic frameworks don't use this type of versioning:
  set_target_properties(drishti
    PROPERTIES
    VERSION "${drishtisdk_VERSION}"
    SOVERSION "${drishtisdk_VERSION_MAJOR}"
    )
endif()

foreach(library ${DRISHTI_LIBS} drishti drishtisdk)
  set_property(TARGET ${library} PROPERTY FOLDER "libs/drishti")
endforeach()

###################
##### install #####
###################

# See: https://cmake.org/Wiki/CMake/Tutorials/How_to_create_a_ProjectConfig.cmake_file

if(DRISHTI_BUILD_SHARED_SDK)
  set(DRISHTI_INSTALL_TARGETS "")  
else()
  set(DRISHTI_INSTALL_TARGETS ${DRISHTI_LIBS})
endif()

# Export minimal public SDK or full internal SDK
set(TARGET_SDK "drishti") 
set(TARGET_PUBLIC_HDRS ${DRISHTI_DRISHTI_HDRS_PUBLIC})

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${drishti_version_config}"
  VERSION "${drishtisdk_VERSION}"
  COMPATIBILITY SameMajorVersion
  )

# Note: variable 'drishti_targets_export_name' used
configure_file("${CMAKE_SOURCE_DIR}/cmake/Config.cmake.in" "${drishti_project_config}" @ONLY)

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

# Experimental size reduction

##################
##### C API ######
##################

if(DRISHTI_BUILD_C_INTERFACE)
  add_library(drishti_c ${API_LIB_TYPE} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISHTI_HDRS_PUBLIC})
  target_compile_definitions(drishti_c PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
  target_link_libraries(drishti_c PRIVATE "${DRISHTI_SDK_LIBS}")
  drishti_hide(drishti_c)
  drishti_strip(drishti_c)
  drishti_symbol_list(drishti_c)
  set_property(TARGET drishti_c PROPERTY FOLDER "libs/drishti")

  if(DRISHTI_BUILD_SHARED_SDK)
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
