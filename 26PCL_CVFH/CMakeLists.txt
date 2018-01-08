cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_CVFH)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_CVFH_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(computeCVFH ${PCL_CVFH_SRC})
 
target_link_libraries(computeCVFH ${PCL_LIBRARIES})
