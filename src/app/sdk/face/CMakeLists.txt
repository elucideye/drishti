#########################
### drishti-face-test ###
#########################

hunter_add_package(nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)

hunter_add_package(aglet)
find_package(aglet CONFIG REQUIRED)

hunter_add_package(ogles_gpgpu)
find_package(ogles_gpgpu CONFIG REQUIRED)

hunter_add_package(Boost COMPONENTS system filesystem)
find_package(Boost CONFIG REQUIRED system filesystem)
set(boost_libs Boost::system Boost::filesystem)

add_executable(drishti-face-test 
  AsyncWorker.h
  FaceTrackerFactoryJson.cpp
  FaceTrackerFactoryJson.h
  FaceTrackerTest.cpp
  FaceTrackerTest.h
  VideoCaptureList.cpp
  VideoCaptureList.h
  drishti-face-test.cpp
)

# https://cmake.org/pipermail/cmake/2012-June/050961.html
# "Bottom line is that Windows does not have an RPATH equivalent.."
# So we can copy any dll to the build tree for launching from IDE

if(WIN32 AND NOT CMAKE_CROSSCOMPILING)
    get_property(drishti_lib_type TARGET drishti::drishti PROPERTY TYPE)
    message("DRISHTI_LIB_TYPE == ${drishti_lib_type}")
    if(drishti_lib_type STREQUAL "SHARED_LIBRARY")
        add_custom_command(
            TARGET drishti-face-test
            PRE_BUILD
            COMMAND
                ${CMAKE_COMMAND} -E
                    copy
                    $<TARGET_FILE:drishti::drishti>
                    $<TARGET_FILE_DIR:drishti-face-test>
        )
    endif()
endif()

target_link_libraries(drishti-face-test PUBLIC ${base_deps} nlohmann_json aglet::aglet ${boost_libs} ogles_gpgpu::ogles_gpgpu)

if(DRISHTI_TEST_BUILD_TESTS)
  target_compile_definitions(drishti-face-test PUBLIC DRISHTI_TEST_BUILD_TESTS=1)
endif()
if(DRISHTI_HUNTER_TEST_ADD_TO_STRING)
  target_compile_definitions(drishti-face-test PUBLIC DRISHTI_HUNTER_TEST_ADD_TO_STRING=1)
endif()
if(DRISHTI_HUNTER_TEST_NO_LOCALECONV)
  target_compile_definitions(drishti-face-test PUBLIC DRISHTI_HUNTER_TEST_NO_LOCALECONV=1)
endif()
install(TARGETS drishti-face-test DESTINATION bin)
