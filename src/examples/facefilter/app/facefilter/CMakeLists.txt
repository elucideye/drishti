if(IOS)
  add_subdirectory(ios)
elseif(ANDROID)
  # See Android Studio project
else()
  add_subdirectory(desktop)
endif()
