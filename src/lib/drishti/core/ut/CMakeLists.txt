# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

if(IOS)
  set(TEST_APP_TYPE "MACOSX_BUNDLE")
endif()

add_executable(test-drishti-core ${TEST_APP_TYPE} test-convert.cpp test-drishti-core.cpp)
target_link_libraries(test-drishti-core drishtisdk ${OpenCV_LIBS} GTest::gtest)
set_property(TARGET test-drishti-core PROPERTY FOLDER "app/tests")

enable_testing()

if(ANDROID)
  drishti_android_add_test(
    NAME DrishtiCoreTest
    COMMAND test-drishti-core
    CLEAN
    )
elseif(IOS)

  find_program(DRISHTI_IOS_DEPLOY_COMMAND ios-deploy)
  if(DRISHTI_IOS_DEPLOY_COMMAND MATCHES "NOTFOUND")
    
  else()
    
  endif()
  
else()
  add_test(NAME DrishtiCoreTest COMMAND test-drishti-core)
endif()
