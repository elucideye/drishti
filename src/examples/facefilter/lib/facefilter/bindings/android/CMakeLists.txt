add_library(facefilter_bindings SHARED ${FACEFILTER_BINDINGS_SRCS}) # ${FACEFILTER_BINDINGS_HDRS}
target_link_libraries(facefilter_bindings PUBLIC facefilter_renderer)
set_target_properties(facefilter_bindings PROPERTIES DEBUG_POSTFIX "")
set_property(TARGET facefilter_bindings PROPERTY FOLDER "examples/lib/")
