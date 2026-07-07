# Settings
cmake_minimum_required(VERSION 3.6.0)
set(CMAKE_CXX_STANDARD  17)


# Link runtime statically to garuntee compatbility
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Debug>:Debug>")
set(CMAKE_MSVC_RUNTIME_LIBRARY "MultiThreaded$<$<CONFIG:Release>:Release>")


# Define project
project(QuadSim)










# Handle complex info in source
add_subdirectory(src)
