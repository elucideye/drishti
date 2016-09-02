if(DRISHTI_BUILD_QT)
  # Some background: https://github.com/ruslo/hunter/issues/253
  # hunter_add_package(Qt COMPONENTS qtmultimedia qtdeclarative)
  # find_package(Qt5Widgets REQUIRED)
  # find_package(Qt5Multimedia REQUIRED)
  # find_package(Qt5MultimediaWidgets REQUIRED)
  # find_package(Qt5OpenGL REQUIRED)
  # find_package(Qt5Qml REQUIRED)
  # find_package(Qt5Quick REQUIRED)
  # find_package(Qt5QuickTest REQUIRED)
  # find_package(Qt5QuickWidgets REQUIRED)

  message("Used Qt:")
  message("  ${QT_ROOT} (root from Hunter)")
  message("  ${Qt5Widgets_DIR} (find_package result)")

  set(CMAKE_AUTOMOC ON)
  set(CMAKE_AUTOUIC ON)
  set(CMAKE_AUTORCC ON)

  add_subdirectory(facefilter)
endif()

