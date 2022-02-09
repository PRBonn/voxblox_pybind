cmake_minimum_required(VERSION 3.10)
project(voxblox)

find_package(Protobuf REQUIRED)

# Compilation options
set(CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)
set(CMAKE_POSITION_INDEPENDENT_CODE ON)
set(CMAKE_BUILD_TYPE Release)
set(CMAKE_CXX_STANDARD 17)

# Disable testing in case any dependency ask for it, we don't test C++ code on this python binding.
option(BUILD_TESTING OFF)

# Build all dependencies as static libraries, so we can't easily pack the pytonn wrapper
option(BUILD_SHARED_LIBS OFF)

# Set some options for the glog library,this could be also passed in the setup.py but I preffer to
# enforce it here so nothing is missing.
option(WITH_GFLAGS OFF)
option(WITH_GTEST OFF)
option(WITH_PKGCONFIG OFF)
option(WITH_TLS OFF)
option(WITH_UNWIND OFF)

# Pull dependencies from the 3rdparty directory
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/glog)
add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/3rdparty/pybind11)

# Include all protobuf_headers and 3rdparty dependencies header files
include(FindVoxbloxProtobuf)
include_directories(${Protobuf_INCLUDE_DIRS})
include_directories(3rdparty/Eigen/)
include_directories(3rdparty/minkindr/minkindr/include/)
include_directories(3rdparty/voxblox/voxblox/include/)

# Generate the necessary protobuf source files and headers
set("VOXBLOX_SRC_ROOT" 3rdparty/voxblox/voxblox/)
voxblox_protobuf_generate_cpp(
  PROTO_SRCS PROTO_HDRS #
  ${VOXBLOX_SRC_ROOT}/proto/voxblox/Block.proto #
  ${VOXBLOX_SRC_ROOT}/proto/voxblox/Layer.proto #
)

# Build the main C++ voxblox library
file(GLOB_RECURSE "VOXBLOX_SRCS" "${VOXBLOX_SRC_ROOT}/src/**/*.cc")
add_library(${PROJECT_NAME} STATIC ${VOXBLOX_SRCS} ${PROTO_SRCS})
add_dependencies(${PROJECT_NAME} protobuf_headers)
target_link_libraries(${PROJECT_NAME} ${Protobuf_LIBRARIES} glog::glog)

# Build the pybind11 wrappers
pybind11_add_module(voxblox_pybind src/voxblox/pybind/voxblox_pybind.cpp)
target_link_libraries(voxblox_pybind PRIVATE ${CMAKE_PROJECT_NAME})
