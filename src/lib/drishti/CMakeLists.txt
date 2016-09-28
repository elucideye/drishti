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
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,--gc-sections")
  else()
    set(CMAKE_SHARED_LINKER_FLAGS "-Wl,-dead_strip")
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
  endif()
  
  add_library(drishti_world ${LIB_TYPE}  ${DRISHTI_WORLD_SOURCES})
  target_link_libraries(drishti_world ${DRISHTI_SDK_3RDPARTY_LIBS})
  target_compile_definitions(drishti_world PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
  if(DRISHTI_USE_TEXT_ARCHIVES)
    target_compile_definitions(drishti_world PUBLIC DRISHTI_USE_TEXT_ARCHIVES=1)
  endif()

  # Define a single drishti_world static library
  set(DRISHTI_LIBS drishti_world)

else()

  ## drishti_core
  add_library(drishti_core ${LIB_TYPE} ${DRISHTI_CORE_SRCS} ${DRISHTI_CORE_HDRS_PUBLIC})
  target_link_libraries(drishti_core PUBLIC ${OpenCV_LIBS})
  if(DRISHTI_USE_TEXT_ARCHIVES)
    target_compile_definitions(drishti_core PUBLIC DRISHTI_USE_TEXT_ARCHIVES=1)
  endif()

  ## drishti_geometry
  add_library(drishti_geometry ${LIB_TYPE} ${DRISHTI_GEOMETRY_SRCS} ${DRISHTI_GEOMETRY_HDRS_PUBLIC})
  target_link_libraries(drishti_geometry PUBLIC ${OpenCV_LIBS})
  target_compile_definitions(drishti_geometry PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio

  ## drishti_sensors
  add_library(drishti_sensor ${LIB_TYPE} ${DRISHTI_SENSOR_SRCS} ${DRISHTI_SENSOR_HDRS_PUBLIC})
  target_link_libraries(drishti_sensor PUBLIC ${OpenCV_LIBS})

  ## drishti_ml
  add_library(drishti_ml ${LIB_TYPE} ${DRISHTI_ML_SRCS} ${DRISHTI_ML_HDRS_PUBLIC})
  target_link_libraries(drishti_ml PUBLIC ${OpenCV_LIBS} ${XGBOOST_LIB})
  target_compile_definitions(drishti_ml PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio

  ## drishti_rcpr
  add_library(drishti_rcpr ${LIB_TYPE} ${DRISHTI_RCPR_SRCS} ${DRISHTI_RCPR_HDRS_PUBLIC})
  target_link_libraries(drishti_rcpr PUBLIC drishti_ml ${OpenCV_LIBS} ${XGBOOST_LIB})

  ## drishti_eye
  add_library(drishti_eye ${LIB_TYPE} ${DRISHTI_EYE_SRCS} ${DRISHTI_EYE_HDRS_PUBLIC})
  target_link_libraries(drishti_eye PUBLIC drishti_ml drishti_rcpr drishti_geometry ${OpenCV_LIBS} ${XGBOOST_LIB})
  if(DRISHTI_USE_TEXT_ARCHIVES)
    target_compile_definitions(drishti_eye PUBLIC DRISHTI_USE_TEXT_ARCHIVES=1)
  endif()

  ## drishti_acf
  if(DRISHTI_BUILD_ACF)
    include_directories(acf/acf/toolbox)
    add_library(drishti_acf ${LIB_TYPE} ${DRISHTI_ACF_SRCS} ${DRISHTI_ACF_HDRS_PUBLIC})
    if(DRISHTI_BUILD_OGLES_GPGPU)
      target_link_libraries(drishti_acf PUBLIC ${OGLES_GPGPU_LIB} ${OpenCV_LIBS})
    endif()
    set(OBJ_ACF drishti_acf)
  endif()

  ## drishti_face
  if(DRISHTI_BUILD_FACE)
    add_library(drishti_face ${LIB_TYPE} ${DRISHTI_FACE_SRCS} ${DRISHTI_FACE_HDRS_PUBLIC})
    target_link_libraries(drishti_face PUBLIC drishti_eye ${OBJ_ACF} ${OpenCV_LIBS})
    set(OBJ_FACE drishti_face)
  endif()

  # Define a list of individual static libraries
  set(DRISHTI_LIBS 
    ${OBJ_ACF}
    ${OBJ_FACE}
    drishti_eye
    drishti_rcpr
    drishti_ml
    drishti_geometry
    drishti_sensor
    drishti_core
    )

  foreach(library ${DRISHTI_LIBS})
    target_link_libraries(${library} PUBLIC Eigen::eigen)
  endforeach()

endif(DRISHTI_BUILD_WORLD)

##
## Unit tests
##

add_subdirectory(drishti)
add_subdirectory(geometry)
if(DRISHTI_BUILD_ACF)
  add_subdirectory(acf)
endif()

include_directories("${PROJECT_BINARY_DIR}")

#
# Build and install a single library or framework from our set of object libraries
#

message("DRISHTISDK_PUBLIC_HDRS ${DRISHTISDK_PUBLIC_HDRS}")

if(IOS)
  set(OPENGL_LIB OpenGLES) 
else()
  set(OPENGL_LIB OpenGL)
endif()

# https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=15038&graph=dependency
# http://www.cmake.org/pipermail/cmake/2014-February/057055.html
# https://public.kitware.com/Bug/bug_relationship_graph.php?bug_id=14970&graph=relation

message("DRISHTI_SDK_3RDPARTY_LIBS --${DRISHTI_SDK_3RDPARTY_LIBS}--")
set(DRISHTI_SDK_SRCS master/drishti_master.hpp master/drishti_master.cpp)

if(ANDROID)
  set(ANDROID_3RDPARTY_LIBS EGL GLESv2)
endif()

if(APPLE)
  set(APPLE_FRAMEWORKS "-framework ${OPENGL_LIB}")
else()
  set(APPLE_FRAMEWORKS "")
endif()

add_library(drishtisdk ${DRISHTI_SDK_SRCS})
target_link_libraries(drishtisdk PUBLIC
  ${DRISHTI_LIBS}
  ${DRISHTI_SDK_3RDPARTY_LIBS}
  ${APPLE_FRAMEWORKS}
  ${ANDROID_3RDPARTY_LIBS}
)

# Build internal lib for console apps, etc.
set_target_properties(drishtisdk
  PROPERTIES
  PUBLIC_HEADER "${DRISHTISDK_PUBLIC_HDRS}"
  XCODE_ATTRIBUTE_COPY_PHASE_STRIP "YES"
  XCODE_ATTRIBUTE_STRIP_INSTALLED_PRODUCT "YES"
  XCODE_ATTRIBUTE_STRIP_STYLE "non-global"
  XCODE_ATTRIBUTE_DEAD_CODE_STRIPPING "YES"
  XCODE_ATTRIBUTE_DEPLOYMENT_POSTPROCESSING "YES"
  )

if(NOT IOS)
  set_target_properties(drishtisdk
    PROPERTIES
    SOVERSION "${drishtisdk_VERSION_MAJOR}"
    VERSION "${drishtisdk_VERSION}"
    )
endif()

############################
###### PUBLIC API ##########
############################

message("=========================================")
message("=========================================")
message("DRISHTI_DRISTHI_HDRS_PUBLIC :  ${DRISHTI_DRISTHI_HDRS_PUBLIC}")
message("=========================================")
message("=========================================")

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
	  AND NOT "${library}" MATCHES "dlib::dlib" 
	  AND NOT "${library}" MATCHES "cereal::cereal"
	  AND NOT "${library}" MATCHES "spdlog::spdlog"
	  AND NOT "${library}" MATCHES "Eigen::eigen") 
    add_custom_command(TARGET drishtisdk POST_BUILD
      COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${library}> "${THIRD_PARTY_INSTALL_DIR}"
      )
  endif()
endforeach()
message("=========================================")
message("=========================================")
message("=========================================")
message("=========================================")

if(DRISHTI_BUILD_SHARED_SDK)
  set(API_LIB_TYPE SHARED)
else()
  set(API_LIB_TYPE STATIC)
endif()

## Always static for ios (we'll build dynamic framework from within xcode)

set(DRISHTI_SDK_LIBS
  ${DRISHTI_LIBS}
  ${DRISHTI_SDK_3RDPARTY_LIBS}   
  ${APPLE_FRAMEWORKS}
  ${ANDROID_3RDPARTY_LIBS}
  )

# Create the public shared library
add_library(drishti ${API_LIB_TYPE} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISTHI_HDRS_PUBLIC})
target_compile_definitions(drishti PUBLIC _USE_MATH_DEFINES) # define M_PI_2 for Visual Studio
target_link_libraries(drishti PRIVATE "${DRISHTI_SDK_LIBS}")
drishti_hide(drishti)
drishti_strip(drishti)

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

foreach(library ${DRISHTI_SDK_3RDPARTY_SRC_LIBS})
  set_property(TARGET ${library} PROPERTY FOLDER "libs/3rdparty")
endforeach()

##################
##### C API ######
##################

if(DRISHTI_BUILD_C_INTERFACE)

  message("DRISHTI_BUILD_C_INTERFACE = ${DRISHTI_BUILD_C_INTERFACE}")
  
  add_library(drishti_c ${API_LIB_TYPE} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISTHI_HDRS_PUBLIC})
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
      EXPORT "${targets_export_name}"
      LIBRARY DESTINATION "lib"
      ARCHIVE DESTINATION "lib"
      RUNTIME DESTINATION "bin"
      INCLUDES DESTINATION "${include_install_dir}"
      )
  endif()
  
endif()


###################
##### install #####
###################

# TODO: The install section is failing for static libraries.
#CMake Error: install(EXPORT "drishtiTargets" ...) includes target "drishti" which requires target "drishti_world" that is not in the export set.
#CMake Error: install(EXPORT "drishtiTargets" ...) includes target "drishti" which requires target "xgboost" that is not in the export set.
#CMake Error: install(EXPORT "drishtiTargets" ...) includes target "drishti" which requires target "boost-pba" that is not in the export set.
if(NOT DRISHTI_BUILD_SHARED_SDK)
  return()
endif()

# Export minimal public SDK or full internal SDK
set(TARGET_SDK "drishti") 
set(TARGET_PUBLIC_HDRS ${DRISHTI_DRISTHI_HDRS_PUBLIC})

message("========== TARGET_PUBLIC_HDRS ${TARGET_PUBLIC_HDRS} ==============")

set(CMAKE_PROJECT_NAME drishti) # overwrite drishtisdk

set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(include_install_dir "include")
set(config_install_dir "share/${CMAKE_PROJECT_NAME}")
set(version_config "${generated_dir}/${CMAKE_PROJECT_NAME}ConfigVersion.cmake")
set(project_config "${generated_dir}/${CMAKE_PROJECT_NAME}Config.cmake")
set(targets_export_name "${CMAKE_PROJECT_NAME}Targets")
set(namespace "${CMAKE_PROJECT_NAME}::")

include(CMakePackageConfigHelpers)
write_basic_package_version_file("${version_config}" VERSION "${drishtisdk_VERSION}" COMPATIBILITY SameMajorVersion)

# Note: variable 'targets_export_name' used
configure_file("${CMAKE_SOURCE_DIR}/cmake/Config.cmake.in" "${project_config}" @ONLY)

install(
  TARGETS ${TARGET_SDK}
  EXPORT "${targets_export_name}"
  LIBRARY DESTINATION "lib"
  ARCHIVE DESTINATION "lib"
  RUNTIME DESTINATION "bin"
  INCLUDES DESTINATION "${include_install_dir}"
  )

install(
  FILES 
  ${TARGET_PUBLIC_HDRS}
  DESTINATION "${include_install_dir}/${CMAKE_PROJECT_NAME}"
  )

install(
  FILES "${project_config}" "${version_config}"
  DESTINATION "${config_install_dir}"
  )

install(
  EXPORT "${targets_export_name}"
  NAMESPACE "${namespace}"
  DESTINATION "${config_install_dir}"
  )


