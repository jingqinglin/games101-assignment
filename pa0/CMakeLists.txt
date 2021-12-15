cmake_minimum_required (VERSION 2.8.11)
project (Transformation)

if (WIN32)
    # Eigen include dir in my Windows device
    include_directories(D:\\Eigen3\\include)
elseif(UNIX AND NOT APPLE)
    include_directories(EIGEN3_INCLUDE_DIR)
endif ()

add_executable (Transformation main.cpp)
