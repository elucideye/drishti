function(drishti_set_unit_test_assets
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

  ### DRISHTI_ACF_FACE_MODEL
  if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner_48x48.cpb")  
    set(${DRISHTI_ACF_FACE_MODEL} "${assets_dir}/drishti_face_inner_48x48.cpb" PARENT_SCOPE)  
  elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner_48x48.pba.z")
    set(${DRISHTI_ACF_FACE_MODEL} "${assets_dir}/drishti_face_inner_48x48.pba.z" PARENT_SCOPE)
  elseif(DRISHTI_SERIALIZE_WITH_CVMATIO AND EXISTS "${assets_dir}/drishti_face_inner_48x48.mat")
    set(${DRISHTI_ACF_FACE_MODEL} "${assets_dir}/drishti_face_inner_48x48.mat" PARENT_SCOPE)
  else()
    message(FATAL_ERROR "Failed to find suitable ACF detector")
  endif()

  ### DRISHTI_FACE_LANDMARKER
  if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_face_inner.cpb")
    set(${DRISHTI_FACE_LANDMARKER} "${assets_dir}/drishti_face_inner.cpb" PARENT_SCOPE)
  elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_face_inner.pba.z") 
    set(${DRISHTI_FACE_LANDMARKER} "${assets_dir}/drishti_face_inner.pba.z" PARENT_SCOPE) 
  else()
    message(FATAL_ERROR "Failed to find suitable face landmarker")
  endif()

  ### DRISHTI_EYE_MODEL
  if(DRISHTI_SERIALIZE_WITH_CEREAL AND EXISTS "${assets_dir}/drishti_eye_full_npd_eix.cpb")
    set(${DRISHTI_EYE_MODEL} "${assets_dir}/drishti_eye_full_npd_eix.cpb" PARENT_SCOPE)
  elseif(DRISHTI_SERIALIZE_WITH_BOOST AND EXISTS "${assets_dir}/drishti_eye_full_npd_eix.pba.z")
    set(${DRISHTI_EYE_MODEL} "${assets_dir}/drishti_eye_full_npd_eix.pba.z" PARENT_SCOPE)
  else()
    message(FATAL_ERROR "Failed to find suitable eye model")
  endif()

  set(${DRISHTI_MEAN_FACE_LANDMARKS} "${assets_dir}/drishti_face_5_point_mean_48x48.xml" PARENT_SCOPE)

endfunction()