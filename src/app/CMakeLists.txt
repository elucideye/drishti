#################
### Libraries ###
#################

add_subdirectory(videoio)

if(DRISHTI_BUILD_FACE)
  add_subdirectory(landmarks)
endif()

####################
### Applications ###
####################

# Top level folder for QT applications:
add_subdirectory(qt)

if(DRISHTI_BUILD_FACE)
  add_subdirectory(facecrop)
endif()

if(DRISHTI_USE_TEXT_ARCHIVES AND DRISHTI_SERIALIZE_WITH_BOOST)
  if(NOT ANDROID AND NOT IOS)
    add_subdirectory(pba)
  endif()
endif()

add_subdirectory(eye)
if(DRISHTI_BUILD_ACF)
  add_subdirectory(acf)
endif()

if(DRISHTI_BUILD_FACE)  
  add_subdirectory(face)
  if(DRISHTI_BUILD_DEST AND DRISHTI_BUILD_EOS AND DRISHTI_BUILD_CV_ML)
    add_subdirectory(pose)
  endif()
endif()

if(DRISHTI_SERIALIZE_WITH_BOOST)
  # TODO: generalize training model serialization
  # Regressors currently use boost .pba.z  
  add_subdirectory(cpr)
  add_subdirectory(dlib)
endif()

add_subdirectory(fddb)

# Currently this has a glfw assumption
# Can use use glfm, sfml or boost for android and iOS
if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU AND (${DRISHTI_DO_GPU_TESTING}))
  add_subdirectory(hci)
endif()
