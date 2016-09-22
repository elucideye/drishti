# Top level folder for QT applications:
add_subdirectory(qt)

if(DRISHTI_USE_TEXT_ARCHIVES)
  if(NOT ANDROID AND NOT IOS)
    add_subdirectory(pba)
  endif()
endif()
