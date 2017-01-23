#
# polly.py --toolchain some-toolchain --fwd drishti_DIR=${DRISHTISDK}/_install/libcxx-hid/share/drishti
#

cmake_minimum_required(VERSION 3.3)
project(DrishtiSample)

# hunter_add_package(drishti) # TODO

# Optionally forward drishti_DIR argument for find_package()
find_package(drishti CONFIG REQUIRED)
find_package(OpenCV REQUIRED)

message(":::::::: :::: :: : OpenCV_DIR == ${OpenCV_DIR}")

add_executable(drishti-example-eye drishti-example-eye.cpp)
target_link_libraries(drishti-example-eye drishti::drishti ${OpenCV_LIBS})
