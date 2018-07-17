add_library(facefilter_bindings ${FACEFILTER_BINDINGS_SRCS}) # ${FACEFILTER_BINDINGS_HDRS}
target_link_libraries(facefilter_bindings PUBLIC facefilter_renderer)
set_property(TARGET facefilter_bindings PROPERTY FOLDER "examples/lib/")
