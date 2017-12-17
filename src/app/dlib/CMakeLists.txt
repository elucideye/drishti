hunter_add_package(nlohmann_json)
find_package(nlohmann_json CONFIG REQUIRED)

################
### RapidXML ###
################

hunter_add_package(RapidXML)
find_package(RapidXML REQUIRED CONFIG)

######################
### drishti-eyexml ###
######################

add_executable(drishti-eyexml eyexml.cpp)
target_include_directories(drishti-eyexml PUBLIC "$<BUILD_INTERFACE:${CMAKE_CURRENT_LIST_DIR}/../>")
target_link_libraries(drishti-eyexml drishtisdk cxxopts::cxxopts RapidXML::RapidXML
  Boost::system
  Boost::filesystem    
  )
set_property(TARGET drishti-eyexml PROPERTY FOLDER "app/console")
install(TARGETS drishti-eyexml DESTINATION bin)

#######################
### drishti-facexml ###
#######################

if(DRISHTI_BUILD_FACE)
  add_executable(drishti-facexml facexml.cpp)
  target_link_libraries(drishti-facexml drishtisdk drishti_landmarks cxxopts::cxxopts RapidXML::RapidXML
    Boost::system
    Boost::filesystem    
    )
  set_property(TARGET drishti-facexml PROPERTY FOLDER "app/console")
  install(TARGETS drishti-facexml DESTINATION bin)
endif()

###################################################
##################### DLIB apps ###################
###################################################

############################
### test_shape_predictor ###
############################

add_executable(drishti_test_shape_predictor test_shape_predictor.cpp)
target_link_libraries(drishti_test_shape_predictor drishtisdk cxxopts::cxxopts PNG::png)
set_property(TARGET drishti_test_shape_predictor PROPERTY FOLDER "app/console")
install(TARGETS drishti_test_shape_predictor DESTINATION bin)

#############################
### train_shape_predictor ###
#############################

if(NOT DRISHTI_BUILD_MIN_SIZE)

  set(train_srcs
    train_shape_predictor.cpp
    RecipeIO.h
    RecipeIO.cpp
    )
  
  add_executable(drishti_train_shape_predictor ${train_srcs})
  target_link_libraries(drishti_train_shape_predictor drishtisdk cxxopts::cxxopts PNG::png nlohmann_json)
  set_property(TARGET drishti_train_shape_predictor PROPERTY FOLDER "app/console")
  install(TARGETS drishti_train_shape_predictor DESTINATION bin)
  
  if(DRISHTI_BUILD_TESTS AND NOT (IOS OR ANDROID))
    set(eye_src_model_private "${DRISHTI_FACES_EYE_MODEL_PRIVATE}")
    set(eye_src_image_private "${DRISHTI_FACES_EYE_IMAGE}")
    
    set(eye_src_model "${CMAKE_CURRENT_BINARY_DIR}/2318-eye.json")
    set(eye_src_image "${CMAKE_CURRENT_BINARY_DIR}/2318-eye.png")
    set(eye_src_train_xml "${CMAKE_CURRENT_BINARY_DIR}/eye_train.xml")

    set(eye_out_model "${CMAKE_CURRENT_BINARY_DIR}/eye.cpb")

    # Rename + copy image + model file side-by-side to meet eyexml requirements:
    #
    # dir/some_eye_0.png
    # dir/some_eye_0.xml
    # dir/some_eye_1.png
    # dir/some_eye_1.xml
    # ...

    configure_file("${eye_src_model_private}" "${eye_src_model}" COPYONLY)
    configure_file("${eye_src_image_private}" "${eye_src_image}" COPYONLY)

    # HOST build train + test
    add_test(
      NAME
      "eye_model_preprocess"
      COMMAND
      "drishti-eyexml"
      "--input=${eye_src_image}"
      "--output=${eye_src_train_xml}"
      "--extension=.json"
      )

    add_test(
      NAME
      "eye_model_train"
      COMMAND
      "drishti_train_shape_predictor"
      "--depth=2"
      "--cascades=3"
      "--dimensions=2,4,6"
      "--train=${eye_src_train_xml}"
      "--model=${eye_out_model}"
      "--silent"        
      )

    add_test(
      NAME
      "eye_model_test"
      COMMAND
      "drishti_test_shape_predictor"
      "--input=${eye_src_train_xml}"
      "--model=${eye_out_model}"
      )

  endif()
endif()
