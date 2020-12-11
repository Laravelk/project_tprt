cmake_minimum_required(VERSION 3.11)
project(tprt)

set(CMAKE_CXX_STANDARD 17)

## eigen
find_package(Eigen3 3.3.9)

##pagmo
#find_package(Pagmo REQUIRED)

##nlopt
find_package(Nlopt REQUIRED)

## Interpolate
find_package(libInterpolate REQUIRED)

## Boost for interpolate lib
find_package(Boost REQUIRED)

## rapidjson
find_package(RapidJSON CONFIG REQUIRED)
include_directories(${RapidJSON_INCLUDE_DIRS})

## vxl
SET(VXL_DIR /Users/ivanmorozov/Work/Lib/project/bin)
find_package(VXL REQUIRED)
if(VXL_FOUND)
    include(${VXL_CMAKE_DIR}/UseVXL.cmake)
else(VXL_FOUND)
    MESSAGE("VXL_DIR should be set to the VXL build directory.")
endif(VXL_FOUND)
if(VXL_CORE_FOUND)
    include_directories(${VXL_CORE_INCLUDE_DIR})
endif(VXL_CORE_FOUND)


message(status, ${rapidjson_FOUND})
add_executable(tprt main.cpp
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
        ReceiverData.cpp
        ReceiverData.hpp
        Horizon/Horizon.h Horizon/GridHorizon.cpp Horizon/GridHorizon.h Derivative.cpp Derivative.h)

target_link_libraries(tprt LINK_PUBLIC ${VXL_CORE_LIBRARIES} NLopt::nlopt vnl vnl_algo Eigen3::Eigen libInterpolate::Interpolate)

