#############
### Boost ###
#############
hunter_add_package(Boost COMPONENTS filesystem system)
find_package(Boost CONFIG REQUIRED filesystem system)  

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

if(DRISHTI_BUILD_FACE)
  add_subdirectory(facecrop)
endif()

add_subdirectory(eye)

# if(DRISHTI_BUILD_ACF)
#   add_subdirectory(acf)
# endif()

if(DRISHTI_BUILD_FACE)  
  add_subdirectory(face)
  if(DRISHTI_BUILD_DEST AND DRISHTI_BUILD_EOS AND DRISHTI_BUILD_CV_ML)
    add_subdirectory(pose)
  endif()
endif()

add_subdirectory(cpr) # cpb
add_subdirectory(dlib) # cpb

add_subdirectory(fddb)

# This application tests the high level drishti::hci modules
# which makes use of OpenGL ES (or compatible) GPGPU processing.
# This is provided by the cross platform aglet library
if(DRISHTI_BUILD_HCI AND DRISHTI_BUILD_OGLES_GPGPU AND aglet_FOUND)
  add_subdirectory(hci)
endif()
