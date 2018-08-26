cmake_minimum_required(VERSION 3.10) # swift support

set(FACEFILTER_RESOURCES
  ${FACEFILTER_XIB_SOURCES}
  ${FACEFILTER_IOS_SHADER_SRCS}
  ${FACEFILTER_IOS_IMAGES}
  )

# add drishti_assets
foreach(resource ${drishti_assets})
  if(NOT EXISTS ${${resource}})
    message(FATAL_ERROR
      "Failed to locate resource ${resource}\n"
      "Perhaps a find_package(drishti_assets) call is missing"
    )
  endif()
  message("ADD RESOURCE: ${resource} ${${resource}}")
  list(APPEND FACEFILTER_RESOURCES ${${resource}})
endforeach()

add_executable(FaceFilter
  ${FACEFILTER_SWIFT_SRCS}
  FaceFilter-Bridging-Header.h  

  # Resources
  ${FACEFILTER_RESOURCES}
)

set(CMAKE_MACOSX_RPATH 1)
# when building, don't use the install RPATH already
# (but later on when installing)
set(CMAKE_BUILD_WITH_INSTALL_RPATH TRUE) 
# add the automatically determined parts of the RPATH
# which point to directories outside the build tree to the install RPATH
set(CMAKE_INSTALL_RPATH_USE_LINK_PATH FALSE)    

set_target_properties(
  FaceFilter
  PROPERTIES
  MACOSX_BUNDLE YES
  MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_LIST_DIR}/Info.plist"
  XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # Universal (iPad + iPhone)
  # http://stackoverflow.com/a/20982506/2288008
  XCODE_ATTRIBUTE_CLANG_ENABLE_OBJC_ARC YES

  # By default this setting is empty => archives not showing up in Xcode organizer.
  # http://stackoverflow.com/a/8102602/2288008y  
  XCODE_ATTRIBUTE_INSTALL_PATH "${CMAKE_BINARY_DIR}/ProductRelease"

  # If this setting is "YES" application can't pass archive validation.
  # http://stackoverflow.com/a/24040412/2288008
  XCODE_ATTRIBUTE_COMBINE_HIDPI_IMAGES "NO" 
  RESOURCE "${FACEFILTER_RESOURCES}"
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER  "com.elucideye.facefilter"

  # dyld: Library not loaded: @rpath/libswiftAVFoundation.dylib
  # Unable to load Swift runtime library (image not found): https://stackoverflow.com/a/24972407a  
  XCODE_ATTRIBUTE_LD_RUNPATH_SEARCH_PATHS "@executable_path/../Frameworks\n@loader_path/Frameworks"
  INSTALL_RPATH "@executable_path/../Frameworks\n@loader_path/Frameworks"

  XCODE_ATTRIBUTE_SWIFT_OBJC_BRIDGING_HEADER "${CMAKE_CURRENT_LIST_DIR}/FaceFilter-Bridging-Header.h"
  
  XCODE_ATTRIBUTE_PRODUCT_NAME "FaceFilter"
  XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER "com.elucideye.facefilter"
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER  "com.elucideye.facefilter"
  XCODE_ATTRIBUTE_GCC_GENERATE_DEBUGGING_SYMBOLS "NO"

  XCODE_ATTRIBUTE_PRODUCT_NAME[variant=Debug] "FaceFilter-Dbg"
  XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER[variant=Debug] "com.elucideye.facefilter.debug"
  XCODE_ATTRIBUTE_PRODUCT_BUNDLE_IDENTIFIER[variant=Debug]  "com.elucideye.facefilter.debug"
  XCODE_ATTRIBUTE_GCC_GENERATE_DEBUGGING_SYMBOLS[variant=Debug] "YES"
  XCODE_ATTRIBUTE_SWIFT_OPTIMIZATION_LEVEL[variant=Debug] "-Onone"
)

if(DRISHTI_OPENGL_ES3)
  # Swift uses Xcode "ACTIVE_COMPILATION_CONDITIONS" for preprocessor flags
  # which have not values (they are either present or absent)
  set_target_properties(
    FaceFilter
    PROPERTIES
    XCODE_ATTRIBUTE_SWIFT_ACTIVE_COMPILATION_CONDITIONS "DRISHTI_OPENGL_ES3"
  )
endif()

if(FACEFILTER_CODE_SIGN)
  set_target_properties(
    FaceFilter
    PROPERTIES
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
  )
endif()

target_link_libraries(FaceFilter PUBLIC 
  facefilter_renderer
  facefilter_bindings
  "-framework CoreMedia"
  "-framework CoreVideo"
  "-framework AVFoundation"
  "-framework UIKit"
  "-framework Foundation"
  "-framework GLKit"
  "-framework OpenGLES"
)

if(TRUE) #DRISHTI_BUILD_SHARED_SDK)
  # DRISHTI_BUILD_SHARED_SDK will only be defined for build tree runs, we need to generalize
  # this to support prebuilt frameworks in builds from src/examples

  set(x_application "FaceFilter")  

  # All further syntax refers to the TARGET (imported or otherwise)
  target_compile_options(${x_application} PUBLIC "-F $<TARGET_FILE_DIR:${x_fmwk_lib}>/../")
  target_link_libraries(${x_application} PUBLIC "-F $<TARGET_FILE_DIR:${x_fmwk_lib}>/../ -framework ${x_fmwk_lib}")
  
  add_custom_command(
    TARGET ${x_application}
    POST_BUILD
    COMMAND
    "${CMAKE_COMMAND}" -E copy_directory
    "$<TARGET_FILE_DIR:${x_fmwk_lib}>/../${x_fmwk_lib}"
    "$<TARGET_FILE_DIR:${x_application}>/Frameworks/${x_fmwk_name}"
    )

  # https://github.com/wrld3d/wrld-example-app/blob/0e4abd90983061788280f5c5b44355f0d11be94f/cmake/templates/ios/EmbedFramework.cmake#L43-L55
  add_custom_command(
    TARGET ${x_application}
    POST_BUILD
    COMMAND
    codesign --force --verbose "$<TARGET_FILE_DIR:${x_application}>/Frameworks/${x_fmwk_name}" --sign "iPhone Developer"
    COMMENT "Signing ${x_fmwk_name}"
  )

  # https://cmake.org/pipermail/cmake/2016-July/063939.html
  set_target_properties(${x_application} PROPERTIES
    XCODE_ATTRIBUTE_CODE_SIGN_IDENTITY "iPhone Developer"
  )
endif()

set_property(TARGET FaceFilter PROPERTY FOLDER "examples/gui")
