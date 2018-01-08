cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_passthrough_filter)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_passthrough_filter_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(passthroughFilter ${PCL_passthrough_filter_SRC})

target_link_libraries (passthroughFilter ${PCL_LIBRARIES})
