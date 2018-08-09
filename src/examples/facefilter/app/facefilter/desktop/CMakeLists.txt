set(test_app facefilter-desktop)

add_executable(
    ${test_app}
    VideoCaptureList.cpp
    VideoCaptureList.h
    face.cpp
)

if(DRISHTI_HAVE_TO_STRING)
  target_compile_definitions(${test_app} PUBLIC DRISHTI_HAVE_TO_STRING=1)
endif()

target_link_libraries(${test_app} PUBLIC facefilter_renderer)

hunter_add_package(cxxopts)
find_package(cxxopts CONFIG REQUIRED)
target_link_libraries(${test_app} PUBLIC cxxopts::cxxopts)

hunter_add_package(aglet)
find_package(aglet CONFIG REQUIRED)
target_link_libraries(${test_app} PUBLIC aglet::aglet)
