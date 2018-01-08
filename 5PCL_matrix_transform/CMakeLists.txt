cmake_minimum_required(VERSION 2.8 FATAL_ERROR)
 
project(PCL_matrix_transform)
 
find_package(PCL 1.7 REQUIRED)
 
include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})
 
set(PCL_BUILD_TYPE Release)
 
file(GLOB PCL_matrix_transform_SRC
    "src/*.h"
    "src/*.cpp"
)
add_executable(matrixTransform ${PCL_matrix_transform_SRC})
 
target_link_libraries(matrixTransform ${PCL_LIBRARIES})
