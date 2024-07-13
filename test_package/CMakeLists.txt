cmake_minimum_required(VERSION 3.15)
project(PackageTest CXX)

find_package(3d_collision_helpers CONFIG REQUIRED)

add_executable(example src/example.cpp)
target_link_libraries(example 3d_collision_helpers::3d_collision_helpers)
