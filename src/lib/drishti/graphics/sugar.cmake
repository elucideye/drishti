# This file generated automatically by:
#   generate_sugar_files.py
# see wiki for more info:
#   https://github.com/ruslo/sugar/wiki/Collecting-sources

if(DEFINED LIB_GRAPHICS_GRAPHICS_SUGAR_CMAKE_)
  return()
else()
  set(LIB_GRAPHICS_GRAPHICS_SUGAR_CMAKE_ 1)
endif()

include(sugar_files)

#sugar_files(
#  DRISHTI_GRAPHICS_SRCS
#  )

sugar_files(
  DRISHTI_GRAPHICS_HDRS_PUBLIC
  drishti_graphics.h
  )

if(DRISHTI_BUILD_OGLES_GPGPU)
  sugar_files(
    DRISHTI_GRAPHICS_SRCS
    LineShader.cpp         
    MeshShader.cpp
    binomial.cpp
    mesh.cpp 
    meshtex.cpp
    saturation.cpp
    )
  sugar_files(
    DRISHTI_GRAPHICS_HDRS_PUBLIC
    GLTexture.h    
    LineShader.h    
    MeshShader.h
    binomial.h
    mesh.h
    meshtex.h
    saturation.h
    )
endif()


