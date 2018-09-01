set_property(GLOBAL PROPERTY USE_FOLDERS ON)

hunter_add_package(sugar)
find_package(sugar CONFIG REQUIRED)

### data ##
hunter_add_package(drishti_assets)
find_package(drishti_assets CONFIG REQUIRED)

set(drishti_assets
  DRISHTI_ASSETS_FACTORY
  DRISHTI_ASSETS_FACE_DETECTOR
  DRISHTI_ASSETS_FACE_LANDMARK_REGRESSOR
  DRISHTI_ASSETS_FACE_DETECTOR_MEAN
  DRISHTI_ASSETS_EYE_MODEL_REGRESSOR
)

### lib ###
sugar_include(lib)
add_subdirectory(lib)

sugar_include(app)
add_subdirectory(app)

if(ANDROID)
  # install resource files, etc
  add_subdirectory(android-studio)
endif()
