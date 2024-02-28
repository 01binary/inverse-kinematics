cmake_minimum_required(VERSION 3.5)
set (CMAKE_CXX_STANDARD 11)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

project (ik-geometric)
add_executable(ik-geometric ik-geometric.cpp)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(ik-geometric Eigen3::Eigen)
