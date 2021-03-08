cmake_minimum_required(VERSION 3.11)
project(tprt)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "-Wall -Wextra")
set(CMAKE_CXX_FLAGS_DEBUG "-g")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")

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
        RayData.cpp
        RayData.h
        Source.cpp
        Source.hpp
        Ray.cpp
        Ray.hpp
        Layer.hpp
        Receiver.cpp
        Receiver.hpp
        Horizon/FlatHorizon.cpp
        Horizon/FlatHorizon.hpp
        Segment.cpp
        Segment.hpp
        Layer.cpp
        VelocityModel.cpp
        VelocityModel.hpp
        Horizon/Horizon.h Horizon/GridHorizon.cpp Horizon/GridHorizon.h Derivative.cpp Derivative.h Optimize.h)

target_link_libraries(tprt LINK_PUBLIC NLopt::nlopt Eigen3::Eigen libInterpolate::Interpolate)

