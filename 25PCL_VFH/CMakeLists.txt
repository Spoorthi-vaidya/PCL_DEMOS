cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_VFH)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_VFH_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(computeVFH ${PCL_VFH_SRC})
 
target_link_libraries(computeVFH ${PCL_LIBRARIES})
