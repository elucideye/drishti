include(sugar_include)
sugar_include(.)

## Customize linker flags 
include(CheckCCompilerFlag)
CHECK_C_COMPILER_FLAG("-Wl,-dead_strip" FLAG_dead_strip)
IF(NOT FLAG_dead_strip)
  SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,--gc-sections")
ELSE()
  SET(CMAKE_SHARED_LINKER_FLAGS "-Wl,-dead_strip")
ENDIF()

set(LIB_TYPE STATIC)

## drishti_core
add_library(drishti_core ${LIB_TYPE} ${DRISHTI_CORE_SRCS} ${DRISHTI_CORE_HDRS_PUBLIC})
target_link_libraries(drishti_core ${OpenCV_LIBS})

## drishti_geometry
add_library(drishti_geometry ${LIB_TYPE} ${DRISHTI_GEOMETRY_SRCS} ${DRISHTI_GEOMETRY_HDRS_PUBLIC})
target_link_libraries(drishti_geometry ${OpenCV_LIBS})

## drishti_sensors
add_library(drishti_sensor ${LIB_TYPE} ${DRISHTI_SENSOR_SRCS} ${DRISHTI_SENSOR_HDRS_PUBLIC})
target_link_libraries(drishti_sensor ${OpenCV_LIBS})

## drishti_ml
add_library(drishti_ml ${LIB_TYPE} ${DRISHTI_ML_SRCS} ${DRISHTI_ML_HDRS_PUBLIC})
target_link_libraries(drishti_ml ${OpenCV_LIBS} ${XGBOOST_LIB})

## drishti_rcpr
add_library(drishti_rcpr ${LIB_TYPE} ${DRISHTI_RCPR_SRCS} ${DRISHTI_RCPR_HDRS_PUBLIC})
target_link_libraries(drishti_rcpr drishti_ml ${OpenCV_LIBS} ${XGBOOST_LIB})

## drishti_eye
add_library(drishti_eye ${LIB_TYPE} ${DRISHTI_EYE_SRCS} ${DRISHTI_EYE_HDRS_PUBLIC})
target_link_libraries(drishti_eye drishti_ml drishti_rcpr ${OpenCV_LIBS} ${XGBOOST_LIB})

## drishti_acf
if(DRISHTI_BUILD_ACF)
  include_directories(acf/acf/toolbox)
  add_library(drishti_acf ${LIB_TYPE} ${DRISHTI_ACF_SRCS} ${DRISHTI_ACF_HDRS_PUBLIC})
  if(DRISHTI_BUILD_OGLES_GPGPU)
    target_link_libraries(drishti_acf ${OGLES_GPGPU_LIB} ${OpenCV_LIBS})
  endif()
  set(OBJ_ACF drishti_acf)
endif()

## drishti_face
if(DRISHTI_BUILD_FACE)
  add_library(drishti_face ${LIB_TYPE} ${DRISHTI_FACE_SRCS} ${DRISHTI_FACE_HDRS_PUBLIC})
  target_link_libraries(drishti_face drishti_eye ${OBJ_ACF} ${OpenCV_LIBS})
  set(OBJ_FACE drishti_face)
endif()

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
  target_link_libraries(${library}
    Eigen::eigen
    )
endforeach()

##
## Unit tests
##

add_subdirectory(drishti)
add_subdirectory(acf)

include_directories("${PROJECT_BINARY_DIR}")
generate_export_header(drishti_core)

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

# get_cmake_property(_variableNames VARIABLES)
# foreach (_variableName ${_variableNames})
#      message(STATUS "VARIABLE: ${_variableName}=${${_variableName}}")
# endforeach()

add_library(drishtisdk ${DRISHTI_SDK_SRCS})
target_link_libraries(drishtisdk PRIVATE
  ${DRISHTI_LIBS}
  ${DRISHTI_SDK_3RDPARTY_LIBS}   
  ${APPLE_FRAMEWORKS}
  ${ANDROID_3RDPARTY_LIBS}
)

# Build internal lib for console apps, etc.
set_target_properties(drishtisdk PROPERTIES PUBLIC_HEADER "${DRISHTISDK_PUBLIC_HDRS}" )
if(NOT IOS)
  set_target_properties(drishtisdk PROPERTIES
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
add_custom_command(TARGET drishtisdk POST_BUILD COMMAND ${CMAKE_COMMAND} -E make_directory "${THIRD_PARTY_INSTALL_DIR}")
foreach(library ${DRISHTI_SDK_3RDPARTY_LIBS})
  if(TARGET ${library}  ## Avoid generator expression errors for header only libs
	  AND NOT "${library}" MATCHES "dlib::dlib" 
	  AND NOT "${library}" MATCHES "cereal::cereal"
	  AND NOT "${library}" MATCHES "spdlog::spdlog"
	  AND NOT "${library}" MATCHES "Eigen::eigen") 
    add_custom_command(TARGET drishtisdk POST_BUILD COMMAND ${CMAKE_COMMAND} -E copy $<TARGET_FILE:${library}> "${THIRD_PARTY_INSTALL_DIR}")
  endif()
endforeach()
message("=========================================")
message("=========================================")
message("=========================================")
message("=========================================")

## Always static for ios (we'll build dynamic framework from within xcode)
set(API_LIB_TYPE SHARED)

add_library(drishti ${API_LIB_TYPE} ${DRISHTI_DRISHTI_SRCS} ${DRISHTI_DRISTHI_HDRS_PUBLIC})

include(drishti_symbols.cmake)

set(DRISHTI_SDK_LIBS
  ${DRISHTI_LIBS}
  ${DRISHTI_SDK_3RDPARTY_LIBS}   
  ${APPLE_FRAMEWORKS}
  ${ANDROID_3RDPARTY_LIBS}
  )

target_link_libraries(drishti PRIVATE "${DRISHTI_SDK_LIBS}")

#add_dependencies(drishti eye_model_rsc)

set_target_properties(drishti PROPERTIES
  CXX_VISIBILITY_PRESET hidden  ### HIDE
  VISIBILITY_INLINES_HIDDEN ON
  XCODE_ATTRIBUTE_GCC_INLINES_ARE_PRIVATE_EXTERN YES 
  XCODE_ATTRIBUTE_GCC_SYMBOLS_PRIVATE_EXTERN YES
  XCODE_ATTRIBUTE_COPY_PHASE_STRIP YES
  PUBLIC_HEADER "${DRISHTI_DRISHTI_HDRS_PUBLIC}"
  )

# if(ANDROID)
#   set_target_properties(drishti PROPERTIES LINK_FLAGS "-static") ### link static dependencies
# endif()

if(NOT IOS)
  set_target_properties(drishti PROPERTIES
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

###################
##### install #####
###################

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

install(FILES "${PROJECT_BINARY_DIR}/drishti_core_export.h" DESTINATION "${include_install_dir}")

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
