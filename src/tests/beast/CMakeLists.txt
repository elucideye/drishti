# Boost ASIO dependencies:
# Boost.System for the boost::system::error_code and boost::system::system_error classes.
# Boost.Regex (optional) if you use any of the read_until() or async_read_until() overloads.
# OpenSSL (optional) if you use Boost.Asio's SSL support.

####################
### image-server ###
####################

add_executable(test-image-server
  image_server.cpp
  "${DRISHTI_3RD_PARTY_DIR}/utilities/websocket_async_server.hpp" # for browsing
  )
target_link_libraries(test-image-server
  ${OpenCV_LIBS}
  Beast::Beast
  cxxopts::cxxopts
  )

if(DRISHTI_USE_IMSHOW)
  target_link_libraries(test-image-server imshow::imshow)
  target_compile_definitions(test-image-server PUBLIC DRISHTI_USE_IMSHOW)
endif()

target_include_directories(test-image-server
  PUBLIC
  "$<BUILD_INTERFACE:${DRISHTI_3RD_PARTY_DIR}/utilities>" # for websocket_async_server.hpp
  "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>" # drishti_stdlib_string.h (android)
  )
set_property(TARGET test-image-server PROPERTY FOLDER "app/tutorial")
install(TARGETS test-image-server DESTINATION bin)

####################
### image-client ###
####################

add_executable(test-image-client image_client.cpp)
target_link_libraries(test-image-client
  PUBLIC
  ${OpenCV_LIBS}
  Beast::Beast
  cxxopts::cxxopts
  )
target_include_directories(test-image-client
  PUBLIC
  "$<BUILD_INTERFACE:${DRISHTI_INCLUDE_DIRECTORIES}>" # drishti_stdlib_string.h (android)
  )
set_property(TARGET test-image-client PROPERTY FOLDER "app/tutorial")
install(TARGETS test-image-client DESTINATION bin)