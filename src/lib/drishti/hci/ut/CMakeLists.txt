# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-hci test-drishti-hci.cpp)
target_link_libraries(test-drishti-hci
  drishtisdk
  ${OpenCV_LIBS}
  GTest::gtest
  ogles_gpgpu::ogles_gpgpu
  ${DRISHTI_OPENGL_LIBS}
  )

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE)
  if(IOS)
    target_link_libraries(test-drishti-hci
      "-framework CoreFoundation"
      "-framework Foundation"  # _NSLog
      )
  else()
    target_link_libraries(test-drishti-hci
      "-framework ImageIO"
      "-framework CoreVideo"
      )
  endif()
endif()

enable_testing()

add_test(NAME DrishtiHciTest COMMAND test-drishti-hci
  "${assets_dir}/drishti_face_inner_48x48.mat"
  "${assets_dir}/drishti_face_5_point_mean_48x48.xml"
  "${assets_dir}/drishti_face_inner.pba.z"
  "${assets_dir}/drishti_eye_full_npd_eix.pba.z"
  )

set_property(TARGET test-drishti-hci PROPERTY FOLDER "app/tests")
