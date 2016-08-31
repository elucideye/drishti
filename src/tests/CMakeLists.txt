# GTEST:
#
# https://github.com/forexample/gtest-pass-filename/blob/master/CMakeLists.txt
# https://github.com/ruslo/hunter/blob/master/examples/GTest/CMakeLists.txt
# https://github.com/ruslo/hunter/wiki/pkg.gtest

hunter_add_package(GTest)
find_package(GTest CONFIG REQUIRED)

add_subdirectory(drishti)

if(DRISHTI_USE_THREAD_POOL_CPP)
  add_subdirectory(thread-pool)
endif()

# QT is used for the portable rendering context
if(DRISHTI_BUILD_QT)
  add_subdirectory(acf)
endif()