###############
### cxxopts ###
###############

# std::regex >= gcc 4.8
hunter_add_package(cxxopts)
find_package(cxxopts CONFIG REQUIRED)

#################
### Libraries ###
#################

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

if(DRISHTI_USE_TEXT_ARCHIVES)
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
