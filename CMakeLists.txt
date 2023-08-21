cmake_minimum_required(VERSION 3.10)
set(CMAKE_CXX_STANDARD 17)  # Set C++17 standard
#compile with gdb debugger
set(CMAKE_CXX_FLAGS_DEBUG "${CMAKE_CXX_FLAGS_DEBUG} -g")
project(kitti_dataset_loader)

find_package(Eigen3 REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(include)
include_directories(${OpenCV_INCLUDE_DIRS})
include_directories(${EIGEN3_INDLUDE_DIRS})

add_library(kitti src/kitti.cpp)

add_executable(kitti_dataset_loader src/main.cpp)

target_link_libraries(kitti_dataset_loader
    kitti
    ${Eigen3_LIBS}
    ${OpenCV_LIBS}
)