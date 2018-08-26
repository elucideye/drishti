# Exclude headers for Android Studio issue:
#add_library(facefilter_renderer ${FACEFILTER_RENDERER_SRCS} ${FACEFILTER_RENDERER_HDRS})
add_library(facefilter_renderer ${FACEFILTER_RENDERER_SRCS})
target_include_directories(
  facefilter_renderer
  PUBLIC
  "$<BUILD_INTERFACE:${LOCAL_INCLUDE_DIRECTORY}>"
)

if(FACEFILTER_HAVE_MAKE_UNIQUE)
  target_compile_definitions(facefilter_renderer PUBLIC FACEFILTER_HAVE_MAKE_UNIQUE)
endif()

set(facefilter_deps "")

### drishti ###
# Note: Don't IWYU here, so we can defer to alias drishti::drishti for local builds
# hunter_add_package(drishti)
# find_package(drishti CONFIG REQUIRED)
if(TARGET drishti::drishti)
  list(APPEND facefilter_deps drishti::drishti)
endif()

### ogles_gpgpu ###
hunter_add_package(ogles_gpgpu)
find_package(ogles_gpgpu CONFIG REQUIRED)
list(APPEND facefilter_deps ogles_gpgpu::ogles_gpgpu)

### OpenCV ###
hunter_add_package(OpenCV)
find_package(OpenCV REQUIRED)
list(APPEND facefilter_deps ${OpenCV_LIBS})

### Boost ###
hunter_add_package(Boost COMPONENTS filesystem system) # FaceTrackerFactoryJson
find_package(Boost CONFIG REQUIRED system filesystem)
list(APPEND facefilter_deps Boost::filesystem Boost::system)

target_link_libraries(facefilter_renderer PUBLIC ${facefilter_deps})
set_property(TARGET facefilter_renderer PROPERTY FOLDER "examples/lib/")

if(DEFINED drishti_FRAMEWORK)
  # This part isn't working, we need to link against the framework for this utility lib too
  target_compile_options(facefilter_renderer PUBLIC "-F $<TARGET_FILE_DIR:${x_fmwk_lib}>/../")
  target_link_libraries(facefilter_renderer PUBLIC "-F $<TARGET_FILE_DIR:${x_fmwk_lib}>/../ -framework ${x_fmwk_lib}")  
endif()
  
