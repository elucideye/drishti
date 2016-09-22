# Top level folder for QT applications:
add_subdirectory(qt)

if(DRISHTI_USE_TEXT_ARCHIVES)
  add_subdirectory(pba)
endif()
