cmake_minimum_required(VERSION 3.11)
project(tprt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3 -g")

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
        Ray/RayData.cpp
        Ray/RayData.h
        Source.cpp
        Source.hpp
        Ray/Ray.cpp
        Ray/Ray.hpp
        Layer.hpp
        Receiver.cpp
        Receiver.hpp
        Horizon/FlatHorizon.cpp
        Horizon/FlatHorizon.hpp
        Layer.cpp
        VelocityModel.cpp
        VelocityModel.hpp
        Horizon/Horizon.h
        Horizon/GridHorizon.cpp
        Horizon/GridHorizon.h
        Math/Derivative.cpp
        Math/Derivative.h
        Optimize.h
        Ray/WaveType.h)

target_link_libraries(tprt LINK_PUBLIC NLopt::nlopt Eigen3::Eigen libInterpolate::Interpolate)

