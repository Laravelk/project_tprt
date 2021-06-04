cmake_minimum_required(VERSION 3.11)
project(tprt)

include(FindOpenMP)
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g ${OpenMP_CXX_FLAGS} -march=native")
set(CMAKE_BUILD_TYPE Release)

#set(CMAKE_CXX_COMPILER /home/laravelk/intel/oneapi/compiler/2021.1.1/linux/bin/icpx)
#set(CMAKE_C_COMPILER /home/laravelk/intel/oneapi/compiler/2021.1.1/linux/bin/icpx)
#set(CMAKE_CXX_FLAGS "-g2")
#set(CMAKE_BUILD_TYPE Debug)
#include(CMakeForceCompiler)
#CMAKE_FORCE_C_COMPILER(icc "Intel C Compiler")
#CMAKE_FORCE_CXX_COMPILER(icpc "Intel C++ Compiler")
#CMAKE_FORCE_Fortran_COMPILER(ifort "Intel Fortran Compiler")

## eigen
find_package(Eigen3)

##nlopt
find_package(NLopt REQUIRED)

## Interpolate
find_package(libInterpolate REQUIRED)

## Boost for interpolate lib
find_package(Boost REQUIRED)

## rapidjson
find_package(RapidJSON CONFIG REQUIRED)
include_directories(${RapidJSON_INCLUDE_DIRS})

message(status, ${rapidjson_FOUND})
add_executable(tprt main.cpp
        Ray/RayData.h
        Data/Source.cpp
        Data/Source.hpp
        Ray/Ray.cpp
        Ray/Ray.hpp
        Data/Layer.hpp
        Data/Receiver.cpp
        Data/Receiver.hpp
        Data/Horizon/FlatHorizon.cpp
        Data/Horizon/FlatHorizon.hpp
        Data/Layer.cpp
        Data/VelocityModel.cpp
        Data/VelocityModel.hpp
        Data/Horizon/Horizon.h
        Data/Horizon/GridHorizon.cpp
        Data/Horizon/GridHorizon.h
        Math/Derivative.h
        Math/Optimize.h
        Ray/WaveType.h
        Math/C_IJ_Matrix.h
        )

target_link_libraries(tprt LINK_PUBLIC NLopt::nlopt Eigen3::Eigen libInterpolate::Interpolate)

