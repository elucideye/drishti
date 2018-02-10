# Copyright (c) 2015, Ruslan Baratov
# Copyright (c) 2015-2016, David Hirvonen
# All rights reserved.

hunter_add_package(nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)

set(app_name facefilter)

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

set(CMAKE_INCLUDE_CURRENT_DIR ON) # for main.moc

find_package(Qt5Quick REQUIRED) # Qt5::Quick
find_package(Qt5Multimedia REQUIRED) # Qt5::QMultimediaDeclarativeModule
find_package(Qt5Qml REQUIRED) # Qt5::QmlQtQuick2Plugin

configure_file(
  facefilter.json
  "${CMAKE_CURRENT_BINARY_DIR}/generated/facefilter.json"
  COPYONLY
  )
configure_file(
  main.qml
  "${CMAKE_CURRENT_BINARY_DIR}/generated"
  COPYONLY
  )

###############################################
### Create a JSON index for each model type ###
###############################################
set(generated_dir "${CMAKE_CURRENT_BINARY_DIR}/generated")
set(FACTORY_NAME "drishti_assets.json")
set(DRISHTI_ASSETS_GENERATED_FACTORY "${generated_dir}/${FACTORY_NAME}")

foreach(model ${DRISHTI_ASSETS_VARS})
  # Create a variable with just the filename (no directory)
  set(model_name "${model}_NAME")
  get_filename_component(${model_name} ${${model}} NAME)
endforeach()

# Use variables:
# * DRISHTI_ASSETS_FACE_DETECTOR_NAME
# * DRISHTI_ASSETS_EYE_MODEL_REGRESSOR_NAME
# * DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR_NAME
# * DRISHTI_ASSETS_FACE_DETECTOR_MEAN_NAME
configure_file(json.cmake.in "${DRISHTI_ASSETS_GENERATED_FACTORY}" @ONLY)

set(facefilter_qrc "${CMAKE_CURRENT_BINARY_DIR}/generated/facefilter.qrc")
set(model_ext "cpb")

# Use variables:
# * DRISHTI_ASSETS_GENERATED_FACTORY
# * DRISHTI_ASSETS_FACE_DETECTOR
# * DRISHTI_ASSETS_EYE_MODEL_REGRESSOR
# * DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR
# * DRISHTI_ASSETS_FACE_DETECTOR_MEAN
configure_file(facefilter.qrc.in "${facefilter_qrc}" @ONLY)

set(
  FACE_FILTER_SOURCES
  main.cpp

  TextureBuffer.hpp
  VideoFilter.hpp
  VideoFilter.cpp
  VideoFilterRunnable.hpp
  VideoFilterRunnable.cpp
  InfoFilter.hpp
  InfoFilter.cpp
  InfoFilterResult.hpp
  InfoFilterRunnable.hpp
  InfoFilterRunnable.cpp
  FPSCalculator.hpp
  FPSCalculator.cpp
  ${facefilter_qrc}
  main.qml
  QtFaceDetectorFactory.h
  QtFaceDetectorFactory.cpp
  QtFaceDetectorFactoryCereal.cpp
  QtFaceMonitor.h
  QtFaceMonitor.cpp
  QMLCameraManager.h
  QMLCameraManager.cpp

  CameraListener.h
  CameraListener.cpp
  
  QtStream.h

  ## Application specific
  FrameHandler.h
  FrameHandler.cpp

  ## Device info
  Device.h
  Device.cpp

  ## Config
  facefilter.json
  )

if(DRISHTI_USE_BEAST)
  list(APPEND FACE_FILTER_SOURCES
    ImageLogger.h
    ImageLogger.cpp
    )
endif()

if(IOS)
  list(APPEND FACE_FILTER_SOURCES Device.mm)
endif()

if(ANDROID)
  
  add_library(${app_name} SHARED ${FACE_FILTER_SOURCES})

  include (GenerateExportHeader)
  generate_export_header(facefilter)

  target_link_libraries(${app_name} 
    Qt5::Quick
    Qt5::Multimedia
    EGL 
    GLESv2
    )

  set_target_properties(
    facefilter
    PROPERTIES
    VERSION 1.0
    )

  hunter_add_package(QtAndroidCMake)
  list(APPEND CMAKE_MODULE_PATH "${QTANDROIDCMAKE_ROOT}")
  include(AddQtAndroidApk)

  add_qt_android_apk(
    TARGET ${app_name}-apk
    BASE_TARGET facefilter
    LAUNCH_TARGET facefilter-launch
    PACKAGE_NAME facefilter.qt
    MANIFEST "${CMAKE_CURRENT_LIST_DIR}/AndroidManifest.xml"
    QML_ROOT "${CMAKE_CURRENT_LIST_DIR}"
    )

  hunter_add_package(Android-SDK)
  message("Path to `android`: ${ANDROID-SDK_ROOT}/android-sdk/tools/android")
  message("Path to `emulator`: ${ANDROID-SDK_ROOT}/android-sdk/tools/emulator")
  message("Path to `adb`: ${ANDROID-SDK_ROOT}/android-sdk/platform-tools/adb")

else()

  # We should load this manually because of cyclic dependencies in Qt
  find_package(Qt5MultimediaWidgets REQUIRED)

  if(IOS OR is_linux OR MSVC)
    set(qt_conf "${CMAKE_CURRENT_LIST_DIR}/qt.conf")
    set(qml_dir "${CMAKE_CURRENT_BINARY_DIR}/qt_qml")

    hunter_add_package(QtQmlManager)
    list(APPEND CMAKE_MODULE_PATH "${QTQMLMANAGER_ROOT}/cmake")
    include(QtCopyQmlTo)
    QtCopyQmlTo(${qml_dir})
  else()
    set(qt_conf "")
    set(qml_dir "")
  endif()

  add_executable(${app_name} MACOSX_BUNDLE ${FACE_FILTER_SOURCES}
    ${qt_conf}
    ${qml_dir}
    )

  if(is_linux OR MSVC)
    # Qt resources are not bundled
    add_custom_command(
        TARGET ${app_name}
        PRE_BUILD
        COMMAND
            ${CMAKE_COMMAND}
            -E
            make_directory
            $<TARGET_FILE_DIR:${app_name}>
        COMMAND
            ${CMAKE_COMMAND}
            -E
            copy_directory
            "${qml_dir}"
            "$<TARGET_FILE_DIR:${app_name}>/qt_qml"
        COMMAND
            ${CMAKE_COMMAND}
            -E
            copy
            "${qt_conf}"
            $<TARGET_FILE_DIR:${app_name}>
    )
  endif()

  if(is_linux)
    configure_file(env.sh ${CMAKE_CURRENT_BINARY_DIR}/env.sh @ONLY)
  endif()

  set_target_properties(
    ${app_name}
    PROPERTIES
    RESOURCE "${qt_conf};${qml_dir}"
    )

  if(IOS)
    set_target_properties(
      ${app_name}
      PROPERTIES
      MACOSX_BUNDLE_INFO_PLIST "${CMAKE_CURRENT_LIST_DIR}/plist.in"
      XCODE_ATTRIBUTE_PRODUCT_NAME
      "${app_name}Example"
      XCODE_ATTRIBUTE_BUNDLE_IDENTIFIER
      "com.github.headupinclouds.drishtisdk.${app_name}"
      XCODE_ATTRIBUTE_TARGETED_DEVICE_FAMILY "1,2" # iPhone/iPad
      )
    target_link_libraries(${app_name} ${Qt5Gui_PLUGINS})
  else()
    find_package(Qt5PrintSupport REQUIRED)
    target_link_libraries(${app_name} Qt5::PrintSupport)
  endif()

  target_link_libraries(${app_name} 
    Qt5::Quick
    Qt5::Multimedia
    Qt5::OpenGL
    Qt5::Sensors
    ### Plugins
    Qt5::QmlQtQuick2Plugin
    ${Qt5Multimedia_PLUGINS}
    Qt5::QMultimediaDeclarativeModule
    )

  ### Workaround for Linux's linker problems {
  set(fix_cyclic FALSE)
  if(APPLE)
    set(fix_cyclic FALSE)
  elseif(CMAKE_COMPILER_IS_GNUCXX)
    # MinGW included
    set(fix_cyclic TRUE)
  elseif(NOT UNIX)
    set(fix_cyclic FALSE)
  elseif("${CMAKE_CXX_COMPILER_ID}" STREQUAL "Clang")
    # Linux Clang
    set(fix_cyclic TRUE)
  endif()

  if(fix_cyclic)
    string(
      REPLACE
      "<LINK_LIBRARIES>"
      " -Wl,--start-group <LINK_LIBRARIES> -Wl,--end-group "
      CMAKE_CXX_LINK_EXECUTABLE
      "${CMAKE_CXX_LINK_EXECUTABLE}"
      )
  endif()
  # }
endif()

target_link_libraries(${app_name} 
  drishtisdk
  ${OGLES_GPGPU_LIB}
  thread-pool-cpp::thread-pool-cpp
  nlohmann_json)

if(DRISHTI_OPENGL_ES3)
  # Note:
  # * QT_OPENGL_ES_3: must be set manually for Android
  # * QT_OPENGL_ES_3 == ON for iOS builds
  target_compile_definitions(${app_name} PUBLIC QT_OPENGL_ES_3=1 DRISHTI_OPENGL_ES3=1)
endif()

if(DRISHTI_USE_BEAST)
  target_compile_definitions(${app_name} PUBLIC DRISHTI_USE_BEAST=1)
  target_link_libraries(${app_name} Beast::Beast)
  target_include_directories(${app_name}
    PUBLIC
    "$<BUILD_INTERFACE:${DRISHTI_3RD_PARTY_DIR}/utilities>" # for websocket_async_server.hpp    
    )
endif()  

set_property(TARGET facefilter PROPERTY FOLDER "app/gui")
