cmake_minimum_required (VERSION 3.3)
project (lrfscpp)

find_package (Eigen3 3.3 REQUIRED NO_MODULE)
SET( EIGEN3_INCLUDE_DIR "C:/eigen-3.4.0" )
IF( NOT EIGEN3_INCLUDE_DIR )
    MESSAGE( FATAL_ERROR "Please point the environment variable EIGEN3_INCLUDE_DIR to the include directory of your Eigen3 installation.")
ENDIF()
INCLUDE_DIRECTORIES ( "${EIGEN3_INCLUDE_DIR}" )

INCLUDE_DIRECTORIES ( "./EigenRand" )
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /arch:AVX2")

add_subdirectory(pybind11)
pybind11_add_module(lrfscpp src/main.cpp)
# target_include_directories(lrfscpp PUBLIC C:/armadillo/include)
