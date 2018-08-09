# add drishti_assets

# https://sites.google.com/a/android.com/tools/tech-docs/new-build-system/user-guide#TOC-Project-Structure
set(assets_dir "${CMAKE_CURRENT_LIST_DIR}/app/src/main/assets")

file(MAKE_DIRECTORY ${assets_dir})

foreach(resource ${drishti_assets})
  if(NOT EXISTS ${${resource}})
    message(FATAL_ERROR
      "Failed to locate resource ${resource}\n"
      "Perhaps a find_package(drishti_assets) call is missing"
    )
  endif()
  message("ADD RESOURCE: ${resource} ${${resource}} ##########################################################")

  configure_file(${${resource}} ${assets_dir} COPYONLY)
endforeach()
