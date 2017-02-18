cmake_minimum_required(VERSION 3.2 FATAL_ERROR)

project(MESH_TRIANGULATION VERSION 0.1 LANGUAGES CXX)

set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -Wall")
set(CMAKE_CXX_FLAGS_RELEASE "${CMAKE_CXX_FLAGS_RELEASE} -Wall")

# Set the output folder where your program will be created
file(GLOB_RECURSE sources src/main/*.cpp src/main/include/*.h)
file(GLOB_RECURSE sources_test src/test/*.cpp)
file(GLOB_RECURSE data resources/*)


#find_package(VTK REQUIRED)
#include(${VTK_USE_FILE})
find_package(PCL REQUIRED)
include_directories("${PCL_INCLUDE_DIRS})" "${PROJECT_SOURCE_DIR}")



link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(mesh ${sources} ${data} ${SOURCE_FILES})

# just for example add some compiler flags
target_compile_options(mesh PUBLIC -std=c++1y -Wall -Wfloat-conversion)

target_include_directories(mesh PUBLIC src/main)

file(COPY ${data} DESTINATION resources)


target_link_libraries(mesh ${PCL_LIBRARIES} ${VTK_LIBRARIES})