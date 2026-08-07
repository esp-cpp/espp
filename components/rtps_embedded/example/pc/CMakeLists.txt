cmake_minimum_required(VERSION 3.20)
project(rtps_embedded_host_pc_example LANGUAGES CXX)

find_package(fastdds REQUIRED COMPONENTS shared)
find_package(OpenSSL REQUIRED)

add_executable(host_pubsub host_pubsub.cpp)
target_compile_features(host_pubsub PRIVATE cxx_std_17)
target_link_libraries(host_pubsub PRIVATE fastdds fastcdr OpenSSL::SSL OpenSSL::Crypto)
