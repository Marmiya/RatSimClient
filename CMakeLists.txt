cmake_minimum_required(VERSION 3.10)
project(RatSim)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(catkin_simple REQUIRED)
catkin_simple(ALL_DEPS_REQUIRED)

list(APPEND CMAKE_PREFIX_PATH "/root/vcpkg/installed/x64-linux")
set(CMAKE_FIND_PACKAGE_PREFER_CONFIG ON)

set(gRPC_DIR "/root/vcpkg/installed/x64-linux/share/grpc")
set(Protobuf_DIR "/root/vcpkg/installed/x64-linux/share/protobuf")
set(OPENSSL_ROOT_DIR "/root/vcpkg/installed/x64-linux")

find_package(PCL CONFIG REQUIRED)
find_package(gRPC CONFIG REQUIRED)
find_package(protobuf CONFIG REQUIRED)
find_package(OpenSSL CONFIG REQUIRED)

# Find additional PCL dependencies
find_package(PkgConfig REQUIRED)
pkg_check_modules(LZ4 REQUIRED liblz4)

# Create the proto library
add_library(ratsim_proto SHARED
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RatSim.pb.cc
  ${CMAKE_CURRENT_SOURCE_DIR}/src/RatSim.grpc.pb.cc
)

target_include_directories(ratsim_proto PUBLIC
  ${CMAKE_CURRENT_SOURCE_DIR}/include
  ${OPENSSL_INCLUDE_DIR}
)

target_link_libraries(ratsim_proto PUBLIC
  protobuf::libprotobuf
  gRPC::grpc++
  gRPC::grpc
  OpenSSL::SSL
  OpenSSL::Crypto
)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${PCL_INCLUDE_DIRS}
  include/RatSim
)

add_executable(ratsim_lidar src/ratsim_lidar.cpp)

target_link_libraries(ratsim_lidar
  ratsim_proto
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  protobuf::libprotobuf
  gRPC::grpc++
  gRPC::grpc
  OpenSSL::SSL
  OpenSSL::Crypto
)

add_executable(send_mesh src/SendMesh.cpp)

target_link_libraries(send_mesh
  ratsim_proto
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  protobuf::libprotobuf
  gRPC::grpc++
  gRPC::grpc
  OpenSSL::SSL
  OpenSSL::Crypto
  ${LZ4_LIBRARIES}
)

add_executable(send_pointcloud src/SendPointCloud.cpp)

target_link_libraries(send_pointcloud
  ratsim_proto
  ${catkin_LIBRARIES}
  ${PCL_LIBRARIES}
  protobuf::libprotobuf
  gRPC::grpc++
  gRPC::grpc
  OpenSSL::SSL
  OpenSSL::Crypto
  ${LZ4_LIBRARIES}
)


cs_install()
cs_export()