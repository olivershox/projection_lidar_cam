cmake_minimum_required(VERSION 3.5)
project(Projection)
 
set(CMAKE_BUILD_TYPE "Release")
set(CMAKE_CXX_FLAGS "-std=c++14")
set(CMAKE_CXX_FLAGS_RELEASE "-O3")
set(CMAKE_CXX_FLAGS_DEBUG "-g -Wall")

find_package(PCL REQUIRED)
find_package(Boost REQUIRED system)
find_package(OpenCV REQUIRED)

link_directories(${OpenCV_LIBRARY_DIRS})
include_directories(${OpenCV_INCLUDE_DIRS})

include_directories(${EIGEN_ROOT})
include_directories(${PROJECT_SOURCE_DIR}/include)
include_directories(${PROJECT_SOURCE_DIR}/src)
include_directories(${PCL_INCLUDE_DIRS})

set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/lib)
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${PROJECT_SOURCE_DIR}/bin)

file(GLOB_RECURSE PARSER_PATH src/*.cpp)
add_library(${PROJECT_NAME} STATIC ${PARSER_PATH})
target_link_libraries(${PROJECT_NAME} libjsoncpp.a ${OpenCV_LIBS} ${Boost_SYSTEM_LIBRARY})

add_executable(projection src/projectOnImage.cpp)
 
target_link_libraries (projection ${PCL_LIBRARIES} ${Boost_LIBRARIES} ${PROJECT_NAME}) 
