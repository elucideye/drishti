#
# polly.py --toolchain some-toolchain --fwd drishti_DIR=${DRISHTISDK}/_install/libcxx-hid/share/drishti
#

cmake_minimum_required(VERSION 3.3)
project(DrishtiSample)

# hunter_add_package(drishti) # TODO
add_executable(drishti-example-eye drishti-example-eye.cpp)

# Optionally forward drishti_DIR argument for find_package()
if(IOS)

  # CMake doesn't like multiple "-F" options, as the second one is automatically
  # stripped, leading to compiler errors:
  # clang: error: cannot specify -o when generating multiple output files  
  
  # target_compile_options(drishti-example-eye
  #   PUBLIC
  #   "-F" "${drishti_DIR}"
  #   "-F" "${OpenCV_DIR}"
  #   )

  # workaround :point_up:
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -F ${drishti_DIR} -F ${OpenCV_DIR}")

  target_link_libraries(drishti-example-eye
    PUBLIC
    "-F ${OpenCV_DIR} -framework opencv2"
    "-F ${drishti_DIR} -framework drishti"
    )

else()

  find_package(drishti CONFIG REQUIRED)
  find_package(OpenCV REQUIRED)
  message(":::::::: :::: :: : OpenCV_DIR == ${OpenCV_DIR}")
  
  target_link_libraries(drishti-example-eye drishti::drishti ${OpenCV_LIBS})
  
endif()
