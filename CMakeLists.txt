cmake_minimum_required (VERSION 2.6)
project (cmpba)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

set(CMAKE_MODULE_PATH ${CMAKE_SOURCE_DIR}/cmake)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
if (EIGEN_FOUND)
  include_directories(${EIGEN_INCLUDE_DIRS})
endif (EIGEN_FOUND)

find_package(Ceres REQUIRED)
include_directories(${CERES_INCLUDE_DIR})

#find_package(Matlab COMPONENTS MX_LIBRARY MEX_COMPILER)

#matlab_add_mex(
#    NAME cmpba_mex
#    SRC "src/cmpba_mex.cpp"
#)



include_directories(
	include
)

add_executable(cba src/projection.cpp src/perspective_projection.cpp src/cba.cpp src/main.cpp)
target_link_libraries (cba Eigen3::Eigen ceres)
