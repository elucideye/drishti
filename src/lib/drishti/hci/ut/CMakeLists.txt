# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-hci test-drishti-hci.cpp)
target_link_libraries(test-drishti-hci
  drishtisdk
  ${OpenCV_LIBS}
  GTest::gtest
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

### DRISHTI_ACF_FACE_MODEL
if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner_48x48.cpb")  
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.cpb")  
elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner_48x48.pba.z")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.pba.z")
elseif(DRISHTI_SERIALIZE_WITH_CVMATIO AND EXISTS "${assets_dir}/drishti_face_inner_48x48.mat")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.mat")
else()
  message(FATAL_ERROR "Failed to find suitable ACF detector")
endif()

### DRISHTI_FACE_LANDMARKER
if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner.cpb")
  set(DRISHTI_FACE_LANDMARKER "${assets_dir}/drishti_face_inner.cpb")
elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner.pba.z") 
  set(DRISHTI_FACE_LANDMARKER "${assets_dir}/drishti_face_inner.pba.z") 
else()
  message(FATAL_ERROR "Failed to find suitable face landmarker")
endif()

### DRISHTI_EYE_MODEL
if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_eye_full_npd_eix.cpb")
  set(DRISHTI_EYE_MODEL "${assets_dir}/drishti_eye_full_npd_eix.cpb")
elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_eye_full_npd_eix.pba.z")
  set(DRISHTI_EYE_MODEL "${assets_dir}/drishti_eye_full_npd_eix.pba.z")
else()
  message(FATAL_ERROR "Failed to find suitable eye model")
endif()

enable_testing()

set(DRISHTI_MEAN_FACE_LANDMARKS "${assets_dir}/drishti_face_5_point_mean_48x48.xml")

add_test(NAME DrishtiHciTest COMMAND test-drishti-hci
  "${DRISHTI_ACF_FACE_MODEL}"
  "${DRISHTI_MEAN_FACE_LANDMARKS}"
  "${DRISHTI_FACE_LANDMARKER}"
  "${DRISHTI_EYE_MODEL}"
  )

set_property(TARGET test-drishti-hci PROPERTY FOLDER "app/tests")
