# found variable?
#include_directories(${GTEST_INCLUDE_DIRS})

add_executable(test-drishti-face test-drishti-face.cpp)
target_link_libraries(test-drishti-face drishtisdk ${OpenCV_LIBS} GTest::gtest)

# OS X Fix: "_CVPixelBufferLockBaseAddress", referenced from: (etc)
if(APPLE AND NOT IOS)
  target_link_libraries(test-drishti-face
    "-framework ImageIO"
    "-framework CoreVideo"
    )
endif()

if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner_48x48.cpb")  
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.cpb")  
elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner_48x48.pba.z")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.pba.z")
elseif(DRISHTI_SERIALIZE_WITH_CVMATIO AND EXISTS "${assets_dir}/drishti_face_inner_48x48.mat")
  set(DRISHTI_ACF_FACE_MODEL "${assets_dir}/drishti_face_inner_48x48.mat")
else()
  message(FATAL_ERROR "Failed to find suitable ACF detector")
endif()

enable_testing()

add_test(NAME DrishtiFaceTest COMMAND test-drishti-face
  "${assets_dir}/drishti_face_inner_48x48.mat"
  "${assets_dir}/drishti_face_5_point_mean_48x48.xml"
  "${DRISHTI_ACF_FACE_MODEL}"
  "${assets_dir}/drishti_eye_full_npd_eix.pba.z"
  )

set_property(TARGET test-drishti-face PROPERTY FOLDER "app/tests")
