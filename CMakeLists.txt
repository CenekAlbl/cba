cmake_minimum_required (VERSION 2.6)
project (cba)

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

add_library(cba SHARED
  src/projection.cpp src/perspective_projection.cpp src/calibrated_perspective_projection.cpp src/cba.cpp
)

target_include_directories(cba PRIVATE .)
target_include_directories(cba PUBLIC ${EIGEN_INCLUDE_DIRS})

set_target_properties(cba PROPERTIES PUBLIC_HEADER include/cba.h)

include(GNUInstallDirs)

install(TARGETS cba
    LIBRARY DESTINATION ${CMAKE_INSTALL_LIBDIR}
    PUBLIC_HEADER DESTINATION ${CMAKE_INSTALL_INCLUDEDIR})


# MAIN
add_executable(run_cba src/main.cpp)

target_link_libraries (run_cba cba Eigen3::Eigen ceres )

# TESTS
add_executable(simple_ba tests/simple_ba_test.cpp tests/generate_scene.h)

target_link_libraries (simple_ba cba Eigen3::Eigen ceres )
