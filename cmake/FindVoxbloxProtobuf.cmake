# This simple cmake module creates a new target "protobuf_headers"

function(VOXBLOX_PROTOBUF_GENERATE_CPP SRCS HDRS)
  # Exported protobuf headers will be here
  include_directories(${CMAKE_CURRENT_BINARY_DIR})

  # Call the original cmake implementation
  protobuf_generate_cpp(PROTO_SRCS PROTO_HDRS ${ARGN})

  # We need to place the generated header files into this directory to avoid changing the original
  # implementation
  set("PROTO_SRC_ROOT" ${CMAKE_CURRENT_BINARY_DIR}/voxblox/)
  file(MAKE_DIRECTORY ${PROTO_SRC_ROOT})
  add_custom_target(
    protobuf_headers
    COMMAND ${CMAKE_COMMAND} -E copy ${PROTO_HDRS} ${PROTO_SRC_ROOT}
    DEPENDS ${PROTO_HDRS})

  # Now just add the protobuf_headers target as a dependency of whatever is using the protobuf
endfunction()
