set(test_app drishti-hci)

hunter_add_package(gauze)
find_package(gauze CONFIG REQUIRED)

set(hci_srcs hci.cpp)
set(hci_defs "")

message("DRISHTI_BUILD_OPENCV_EXTRA=${DRISHTI_BUILD_OPENCV_EXTRA}")
if(DRISHTI_BUILD_OPENCV_EXTRA)
  list(APPEND hci_srcs FacePatchTracker.h FacePatchTracker.cpp)
  list(APPEND hci_defs DRISHTI_BUILD_OPENCV_CONTRIB=1)
endif()

add_executable(${test_app} ${hci_srcs})
target_link_libraries(${test_app} PUBLIC
  drishtisdk
  cxxopts::cxxopts
  ${OpenCV_LIBS}
  drishti_videoio
  aglet::aglet
  gauze::gauze)

foreach(def ${hci_defs})
  target_compile_definitions(${test_app} PUBLIC ${def})
endforeach()

# Get toolchain suffix:
if(EXISTS "${CMAKE_TOOLCHAIN_FILE}")
  get_filename_component(toolchain_suffix "${CMAKE_TOOLCHAIN_FILE}" NAME_WE)
else()
  set(toolchain_suffix "default")
endif()

if(DRISHTI_IS_MOBILE)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_IS_MOBILE=1)
  if(IOS)
    set(BUNDLE_ID "com.elucideye.drishti.${toolchain_suffix}.${test_app}")
    message("BUNDLE_ID = ${BUNDLE_ID}")
    set_target_properties(
      "${test_app}"
      PROPERTIES
      MACOSX_BUNDLE_GUI_IDENTIFIER "${BUNDLE_ID}"
      )
  endif()
endif()

target_include_directories(${test_app} PUBLIC "$<BUILD_INTERFACE:${DRISHTI_APP_DIRECTORIES}>")
set_property(TARGET ${test_app} PROPERTY FOLDER "app/console")
install(TARGETS ${test_app} DESTINATION bin)
