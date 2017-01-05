function(drishti_set_unit_test_assets
    DRISHTI_ACF_FACE_MODEL
    DRISHTI_FACE_LANDMARKER
    DRISHTI_EYE_MODEL
    DRISHTI_MEAN_FACE_LANDMARKS
    )

  set(ext "")
  if(DRISHTI_SERIALIZE_WITH_CEREAL)
    set(ext "cpb")
  elseif(DRISHTI_SERIALIZE_WITH_BOOST AND NOT MSVC)
    set(ext "pba.z")
  elseif(DRISHTI_SERIALIZE_WITH_BOOST AND DRISHTI_USE_TEXT_ARCHIVES)
    set(ext "txt")
  elseif(DRISHTI_SERIALIZE_WITH_CVMATIO)
    set(ext "mat")
  else()
    message(FATAL_ERROR "Invalid configuration")
  endif()

  ### DRISHTI_ACF_FACE_MODEL
  set(file "${assets_dir}/drishti_face_inner_48x48.${ext}")
  set("${DRISHTI_ACF_FACE_MODEL}" "${file}" PARENT_SCOPE)

  if(NOT EXISTS "${file}")
    message(FATAL_ERROR "File not found: '${file}'")
  endif()

  ### DRISHTI_FACE_LANDMARKER
  set(file "${assets_dir}/drishti_face_inner.${ext}")
  set("${DRISHTI_FACE_LANDMARKER}" "${file}" PARENT_SCOPE)

  if(NOT EXISTS "${file}")
    message(FATAL_ERROR "File not found: '${file}'")
  endif()

  ### DRISHTI_EYE_MODEL
  set(file "${assets_dir}/drishti_eye_full_npd_eix.${ext}")
  set("${DRISHTI_EYE_MODEL}" "${file}" PARENT_SCOPE)

  if(NOT EXISTS "${file}")
    message(FATAL_ERROR "File not found: '${file}'")
  endif()

  ### DRISHTI_MEAN_FACE_LANDMARKS
  set(file "${assets_dir}/drishti_face_5_point_mean_48x48.xml")
  set("${DRISHTI_MEAN_FACE_LANDMARKS}" "${file}" PARENT_SCOPE)

  if(NOT EXISTS "${file}")
    message(FATAL_ERROR "File not found: '${file}'")
  endif()
endfunction()
