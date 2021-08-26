cmake_minimum_required(VERSION 3.10)
project(vo)

set(CMAKE_CXX_FLAGS "-O2 -march=native -mtune=native -mavx")

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(THREADS_PREFER_PTHREAD_FLAG ON)

find_package(Eigen3)
find_package(Ceres)

include_directories(${EIGEN_INCLUDE_DIR} src/include)

add_executable(vo_3d3d src/include/matrix.cc 
					   src/include/evaluate.cc 
					   src/include/io_utils.cc 
					   src/include/math_utils.cc 
					   src/vo_3d3d.cc)

add_executable(vo_3d2d src/include/matrix.cc 
					   src/include/evaluate.cc 
					   src/include/io_utils.cc 
					   src/include/math_utils.cc 
					   src/vo_3d2d.cc)

target_link_libraries(vo_3d2d ${CERES_LIBRARIES})
target_link_libraries(vo_3d3d ${CERES_LIBRARIES})