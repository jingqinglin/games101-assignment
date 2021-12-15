cmake_minimum_required(VERSION 3.10)
project(Rasterizer)

find_package(OpenCV REQUIRED)

set(CMAKE_CXX_STANDARD 17)

if (WIN32)
    # Eigen include dir in my Windows device
    include_directories(D:\\Eigen3\\include)
elseif(UNIX AND NOT APPLE)
    include_directories(/usr/local/include)
endif ()

add_executable(Rasterizer main.cpp rasterizer.hpp rasterizer.cpp global.hpp Triangle.hpp Triangle.cpp)
target_link_libraries(Rasterizer ${OpenCV_LIBRARIES})
