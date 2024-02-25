cmake_minimum_required(VERSION 3.5)

find_package(Eigen3 3.4 REQUIRED NO_MODULE)
include_directories(${EIGEN3_INCLUDE_DIR})

project (gradient_descent)
add_executable(gradient_descent ik.cpp)

# https://eigen.tuxfamily.org/dox/TopicCMakeGuide.html
target_link_libraries(gradient_descent Eigen3::Eigen)
