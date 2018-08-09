if(DEFINED APP_FACEFILTER_IOS_SHADERS_SUGAR_CMAKE_)
  return()
else()
  set(APP_FACEFILTER_IOS_SHADERS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

# Requires:
# @FACEFILTER_SWIZZLE@@
if(DRISHTI_OPENGL_ES3)
  set(FACEFILTER_SWIZZLE "ra")
else()
  set(FACEFILTER_SWIZZLE "rg")
endif()
set(shader_fsh_in "${CMAKE_CURRENT_LIST_DIR}/Shader.fsh.in")
set(shader_fsh "${CMAKE_CURRENT_BINARY_DIR}/Shader.fsh")
configure_file("${shader_fsh_in}" "${shader_fsh}" @ONLY)

sugar_files(
    FACEFILTER_IOS_SHADER_SRCS
    "${shader_fsh}"
    Shader.vsh
)
